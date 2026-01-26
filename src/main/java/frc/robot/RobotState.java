package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.generated.TunerConstants;


public class RobotState {

    private static final double poseBufferSizeSec = 2.0;
    private static final double turretAngleBufferSizeSec = 2.0;



    private Pose2d estimatedPose = Pose2d.kZero;
    private Pose2d odometryPose = Pose2d.kZero;
      private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
     private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
        TimeInterpolatableBuffer.createBuffer(turretAngleBufferSizeSec);


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

}
