package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static frc.robot.Constants.ShooterK.*;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.util.AllianceFlipUtil;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose2dLogger;

import static frc.util.GeomUtil.*;



//TODO: Make sure to go back and comment effectively line by line to ensure that youngins are able to understand (this code is basically self explanatory so long as you understand sotm)
public class ShotCalculation {
    private Rotation2d m_lastTurretAngle;
    private double m_lastHoodAngle;
    private Rotation2d m_turretAngle;
    private double m_hoodAngle = Double.NaN; //make sure that we dont set angle to zero :skull:
    private double m_turretVelocity;
    private double m_hoodVelocity;

    //TODO: Change the 0.02 to whatever our loop time will be
    private final LinearFilter m_turretAngleFilter = 
        LinearFilter.movingAverage((int)(0.1 / 0.02));
    private final LinearFilter m_hoodAngleFilter = 
        LinearFilter.movingAverage((int) (0.1 / 0.02));

    private static double m_minDistance;
    private static double m_maxDistance;
    private static double m_phaseDelay;

    private static InterpolatingTreeMap<Double, Rotation2d> m_shotHoodAngleMap = 
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static InterpolatingDoubleTreeMap m_shotFlywheelSpeedMap = 
        new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap m_timeOfFlightMap = 
        new InterpolatingDoubleTreeMap();

    private final Pose2dLogger log_lookaheadPose =  WaltLogger.logPose2d(kLogTab, "lookaheadPose");
    private final DoubleLogger log_turretToTargetDistance = WaltLogger.logDouble(kLogTab, "turretToTargetDistance");

    
    
    public record ShootingParameters (
        boolean isValid,
        Rotation2d turretAngle,
        double turretVelocity,
        double hoodAngle,
        double hoodVelocity,
        double flywheelSpeed){}

    
    private ShootingParameters latestParameters = null;

    
    //lookup table produced from MA 1/20/26
    static {
        m_minDistance = 1.17;
        m_maxDistance = 5.63;
        m_phaseDelay = 0.05; //dummy value
    
        m_shotHoodAngleMap.put(1.17, Rotation2d.fromDegrees(19.0));
        m_shotHoodAngleMap.put(1.55, Rotation2d.fromDegrees(19.0));
        m_shotHoodAngleMap.put(1.96, Rotation2d.fromDegrees(19.0));
        m_shotHoodAngleMap.put(2.14, Rotation2d.fromDegrees(20.0));
        m_shotHoodAngleMap.put(2.40, Rotation2d.fromDegrees(21.0));
        m_shotHoodAngleMap.put(2.78, Rotation2d.fromDegrees(22.0));
        m_shotHoodAngleMap.put(3.10, Rotation2d.fromDegrees(24.0));
        m_shotHoodAngleMap.put(3.44, Rotation2d.fromDegrees(26.0));
        m_shotHoodAngleMap.put(3.84, Rotation2d.fromDegrees(27.0));
        m_shotHoodAngleMap.put(4.60, Rotation2d.fromDegrees(32.0));
        m_shotHoodAngleMap.put(5.12, Rotation2d.fromDegrees(33.0));
        m_shotHoodAngleMap.put(5.63, Rotation2d.fromDegrees(36.0));
    
        m_shotFlywheelSpeedMap.put(1.17, 185.0);
        m_shotFlywheelSpeedMap.put(1.55, 205.0);
        m_shotFlywheelSpeedMap.put(1.96, 225.0);
        m_shotFlywheelSpeedMap.put(2.14, 225.0);
        m_shotFlywheelSpeedMap.put(2.40, 225.0);
        m_shotFlywheelSpeedMap.put(2.78, 230.0);
        m_shotFlywheelSpeedMap.put(3.10, 235.0);
        m_shotFlywheelSpeedMap.put(3.44, 238.0);
        m_shotFlywheelSpeedMap.put(3.84, 245.0);
        m_shotFlywheelSpeedMap.put(4.60, 252.0);
        m_shotFlywheelSpeedMap.put(5.12, 265.0);
        m_shotFlywheelSpeedMap.put(5.63, 270.0);
    
        m_timeOfFlightMap.put(5.68, 1.16);
        m_timeOfFlightMap.put(4.55, 1.12);
        m_timeOfFlightMap.put(3.15, 1.11);
        m_timeOfFlightMap.put(1.88, 1.09);
        m_timeOfFlightMap.put(1.38, 0.90);
    }

    /**
     * Gets the parameters that are required to shoot on the fly, accounting for robot velocity, shooter velocity, and distance
     * @
     */
    public ShootingParameters getParameters() {
        //if there already exists parameters that still apply, wont generate new params
        if (latestParameters != null) {
            return latestParameters;
        }

        /* CALCULATING ESTIMATED POSE */
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
        ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
        estimatedPose =
            estimatedPose.exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * m_phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * m_phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * m_phaseDelay));

        
        /* CALCULATING DISTANCE FROM TURRET TO TARGET */
        Translation2d target = 
            AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Pose2d turretPosition = estimatedPose.transformBy(toTransform2d(kRobotToTurret));
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());


        /* CALCULATING FIELD RELATIVE TURRET VELOCITY */
        ChassisSpeeds robotVelocity = RobotState.getInstance().getRobotVelocity();
        double robotAngleRad = estimatedPose.getRotation().getRadians();
        double turretVelocityX = 
            robotVelocity.vxMetersPerSecond 
                + robotVelocity.omegaRadiansPerSecond 
                    * (kRobotToTurret.getX() * Math.cos(robotAngleRad)
                            -kRobotToTurret.getX() * Math.sin(robotAngleRad));
        double turretVelocityY = 
            robotVelocity.vyMetersPerSecond 
                + robotVelocity.omegaRadiansPerSecond 
                    * (kRobotToTurret.getY() * Math.cos(robotAngleRad)
                            -kRobotToTurret.getY() * Math.sin(robotAngleRad));

        //accounting for velocity offset (ball shoot, robot move, there will be offset created by that movement therefore we NEED to account)
        double timeOfFlight; 
        Pose2d lookaheadPose = turretPosition; //sort of a "guess" position
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

        //loop to converge onto one specific value
        for (int i = 0; i < 20; i++) {
            timeOfFlight = m_timeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose = 
                new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation()
                );
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        /* calculate the parameters for shooting while compensating for robot velocity */

        m_turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        m_hoodAngle = m_shotHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();

        //ensure that our shooter subsystem angles are NOT empty
        if (m_lastTurretAngle == null)
            m_lastTurretAngle = m_turretAngle;
        if (Double.isNaN(m_lastHoodAngle))
            m_lastHoodAngle = m_hoodAngle;

            
        //TODO: Change the 0.02 to whatever our loop time will be
        m_turretVelocity = 
            m_turretAngleFilter.calculate(
                m_turretAngle.minus(m_lastTurretAngle).getRadians() / 0.02);
        m_hoodVelocity = 
            m_hoodAngleFilter.calculate(
                (m_hoodAngle - m_lastHoodAngle) / 0.02);

        m_lastTurretAngle = m_turretAngle;
        m_lastHoodAngle = m_hoodAngle;

        latestParameters =
            new ShootingParameters(
                lookaheadTurretToTargetDistance >= m_minDistance
                    && lookaheadTurretToTargetDistance <= m_maxDistance,
                m_turretAngle,
                m_turretVelocity,
                m_hoodAngle, 
                m_hoodVelocity, 
                m_shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

        log_lookaheadPose.accept(lookaheadPose);
        log_turretToTargetDistance.accept(turretToTargetDistance);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
}
