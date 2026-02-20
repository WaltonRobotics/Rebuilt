package frc.robot;

import java.util.NoSuchElementException;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;


public class RobotState {

    private static final double poseBufferSizeSec = 2.0;
    private static final double turretAngleBufferSizeSec = 2.0;
    private static final Matrix<N3, N1> odometryStateStdDevs = 
        new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));


    private Pose2d estimatedPose = Pose2d.kZero;
    private Pose2d odometryPose = Pose2d.kZero;
      private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
     private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
        TimeInterpolatableBuffer.createBuffer(turretAngleBufferSizeSec);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    private final SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
    private Rotation2d gyroOffset = Rotation2d.kZero;

    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private RobotState() {
        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
        }
        kinematics = new SwerveDriveKinematics(TunerConstants.moduleTranslations);
    }


    /**
     * Reset the pose estimation and the odometry estimation to the given pose
     */
    public void resetPose(Pose2d pose) {
        gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
        estimatedPose = pose;
        odometryPose = pose;
        poseBuffer.clear();
    }

    public Rotation2d getRotation() {
        return estimatedPose.getRotation();
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public ChassisSpeeds getRobotVelocity() {
        return robotVelocity;
    }

    public void setRobotVelocity(ChassisSpeeds robotVelocity) {
        this.robotVelocity = robotVelocity;
    }
    
    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
    }

        /** Adds a new odometry sample from the drive subsystem. */
    public void addOdometryObservation(OdometryObservation observation) {
        // Update odometry pose
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist);

        // Replace odometry pose with gyro if present
        observation.gyroAngle.ifPresent(
            gyroAngle -> {
            // Add offset to measured angle
            Rotation2d angle = gyroAngle.plus(gyroOffset);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
            });

        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);

        // Apply odometry delta to vision pose estimate
        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    /** Adds a turret pose observation from the turret subsystem */
    public void addTurretObservation(TurretObservation observation) {
        turretAngleBuffer.addSample(observation.timestamp(), observation.turretAngle);
    }

    /** Adds a new vision pose observation from the vision subsystem. */
    public void addVisionObservation(VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
        if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
            return;
        }
        } catch (NoSuchElementException ex) {
        return;
        }

        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
        // exit if not there
        return;
        }

        // Calculate transforms between odometry pose and vision sample pose
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

        // Shift estimated pose backwards to sample time
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
        r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
        double stdDev = qStdDevs.get(row, 0);
        if (stdDev == 0.0) {
            visionK.set(row, row, 0.0);
        } else {
            visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
        }
        }

        // Calculate the transform from the shifted estimate to the observation pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

        // Scale the transform by the Kalman gain
        var kTimesTransform =
            visionK.times(
                VecBuilder.fill(
                    transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform =
            new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // Recalculate the current estimate by applying the scaled transform to the old estimate
        // then shifting forwards using odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    // MARK: - Type declarations

    public record OdometryObservation(
        double timestamp, SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle) {}

    public record VisionObservation(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {}

    public record TurretObservation(double timestamp, Rotation2d turretAngle) {}

}
