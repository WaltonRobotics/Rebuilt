package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.ShooterK.*;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.VisionSim;
import frc.util.AllianceFlipUtil;
import frc.util.GeomUtil;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose2dLogger;

import static frc.util.GeomUtil.*;
//MA's exact code
//TODO: Make sure to go back and look over 5000s code as a sanity check to makesure that im doing everything correctly
public class ShotCalculator {
    private static ShotCalculator instance;

    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

    private LinearVelocity m_fuelPathVelocity = MetersPerSecond.of(9.1); //TODO: pester build again to ask what the max velocity our ball can shoot and the min velocity to guesstimate this val?

    private Rotation2d lastTurretAngle;
    private double lastHoodAngle;
    private Rotation2d turretAngle;
    private double hoodAngle = Double.NaN;
    private double turretVelocity;
    private double hoodVelocity;
    private final Swerve m_drivetrain;

    private final Pose2dLogger log_estimatedPose = WaltLogger.logPose2d(kLogTab, "estimatedPose");
    private final Pose2dLogger log_lookaheadPose = WaltLogger.logPose2d(kLogTab, "lookaheadPose");
    private final DoubleLogger log_turretToTargetDistance = WaltLogger.logDouble(kLogTab, "turretToTargetDistance");

    public ShotCalculator(Swerve drivetrain) {
        m_drivetrain = drivetrain;
    }

    public record ShootingParameters(
            boolean isValid,
            Rotation2d turretAngle,
            double turretVelocity,
            double hoodAngle,
            double hoodVelocity,
            double flywheelSpeed) {
    }

    // Cache parameters
    private ShootingParameters latestParameters = null;

    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;
    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        minDistance = 1.34;
        maxDistance = 10000; //testing time
        phaseDelay = 0.03;

        shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
        shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
        shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
        shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
        shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
        shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
        shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
        shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

        shotFlywheelSpeedMap.put(1.34, 210.0);
        shotFlywheelSpeedMap.put(1.78, 220.0);
        shotFlywheelSpeedMap.put(2.17, 220.0);
        shotFlywheelSpeedMap.put(2.81, 230.0);
        shotFlywheelSpeedMap.put(3.82, 250.0);
        shotFlywheelSpeedMap.put(4.09, 255.0);
        shotFlywheelSpeedMap.put(4.40, 260.0);
        shotFlywheelSpeedMap.put(4.77, 265.0);
        shotFlywheelSpeedMap.put(5.57, 275.0);
        shotFlywheelSpeedMap.put(5.60, 290.0);

        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);
    }

    public ShootingParameters getParameters() {
        if (latestParameters != null) {
            return latestParameters;
        }

        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = m_drivetrain.getState().Pose;
        ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
        estimatedPose = estimatedPose.exp(
                new Twist2d(
                        robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate distance from turret to target
        Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Pose2d turretPosition = estimatedPose.transformBy(GeomUtil.toTransform2d(kRobotToTurret));
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX = robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (kRobotToTurret.getY() * Math.cos(robotAngle)
                                - kRobotToTurret.getX() * Math.sin(robotAngle));
        double turretVelocityY = robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (kRobotToTurret.getX() * Math.cos(robotAngle)
                                -kRobotToTurret.getY() * Math.sin(robotAngle));

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
        if (lastTurretAngle == null)
            lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle))
            lastHoodAngle = hoodAngle;
        turretVelocity = turretAngleFilter.calculate(
                turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
        hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;
        latestParameters = new ShootingParameters(
                lookaheadTurretToTargetDistance >= minDistance
                        && lookaheadTurretToTargetDistance <= maxDistance,
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

        log_estimatedPose.accept(estimatedPose);
        log_lookaheadPose.accept(lookaheadPose);
        log_turretToTargetDistance.accept(turretToTargetDistance);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
    public LinearVelocity getFuelPathVelocity () {
        return m_fuelPathVelocity;
    }
}
