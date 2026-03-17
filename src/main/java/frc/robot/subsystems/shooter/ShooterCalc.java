package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.WpiK;
import frc.robot.generated.TunerConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShotCalculator.ShotData;
import frc.util.AllianceFlipUtil;
import frc.util.AllianceZoneUtil;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose3dLogger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterK.*;

public class ShooterCalc {
    private final Supplier<SwerveDriveState> m_threadsafeSwerveDriveStateSup;
    private final Supplier<Angle> m_turretPosSup;
    private final SwerveDriveKinematics m_swerveKinematics = new SwerveDriveKinematics(TunerConstants.moduleTranslations);

    private final Pose3dLogger log_globalShotTarget = WaltLogger.logPose3d("ShotCalc", "globalTarget");
    private final Pose3dLogger log_calculatedShotTarget = WaltLogger.logPose3d("ShotCalc", "shotCalcTarget");
    private final DoubleLogger log_rawDesiredTurretRot = WaltLogger.logDouble("ShotCalc", "rawDesiredTurretRots");
    private final DoubleLogger log_desiredTurretRot = new DoubleLogger("ShotCalc", "desiredTurretRotations");
    private final Pose3dLogger log_desiredAimPose = WaltLogger.logPose3d("ShotCalc", "DesiredAimPose");
    private final Pose3dLogger log_currentAimPose = WaltLogger.logPose3d("ShotCalc", "CurrentAimPose");
    private final DoubleLogger log_loopTime = WaltLogger.logDouble("ShotCalc", "LoopTimeMsec");


    // Precomputed doubles for hot-path unit conversions
    private static final double kTurretMinRotsD = kTurretMinRots.in(Rotations);
    private static final double kTurretMinRotsMagnitudeD = kTurretMinRots.magnitude();
    private static final double kTurretMaxRotsD = kTurretMaxRots.in(Rotations);
    private static final double kTurretMaxRotsMagnitudeD = kTurretMaxRots.magnitude();

    private final ShotData kEmptyShotData = new ShotData(0, 0);
    private final AzimuthCalcDetails kEmptyAzimuthCalcDetails = new AzimuthCalcDetails(Rotations.zero(), new Pose3d(), new Pose3d(), 0, RotationsPerSecond.zero());
    private final ShotCalcOutputs kEmptyShotCalcOutputs = new ShotCalcOutputs(kEmptyAzimuthCalcDetails, kEmptyShotData, Rotation.zero(), Rotation.zero(), RotationsPerSecond.zero());

    private boolean m_useSotm = false;
    private Translation3d m_aimTarget = Translation3d.kZero;
    private ShotCalcOutputs m_shotCalcOutputs = kEmptyShotCalcOutputs;
    
    private final Notifier m_notifier = new Notifier(this::calcCallback);
    private final Timer m_calcTimer = new Timer();

    public ShooterCalc(Supplier<SwerveDriveState> threadsafeSwerveDriveStateSup, Supplier<Angle> turretPosSup) {
        m_threadsafeSwerveDriveStateSup = threadsafeSwerveDriveStateSup;
        m_turretPosSup = turretPosSup;

        m_notifier.setName("ShooterCalc");
        m_notifier.startPeriodic(Hertz.of(25)); // 2x slower than robot loop
    }

    public void shouldUseSotm(boolean should) {
        m_useSotm = should;
    }

    public Translation3d getLatestAimTarget() {
        return m_aimTarget;
    }

    public ShotCalcOutputs getLatestShotCalcOutputs() {
        return m_shotCalcOutputs;
    }

    private void calcCallback() {
        m_calcTimer.restart();
        // getters from outside
        SwerveDriveState swerveState = m_threadsafeSwerveDriveStateSup.get();
        Pose2d robotPose = swerveState.Pose;
        ChassisSpeeds robotChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_swerveKinematics.toChassisSpeeds(swerveState.ModuleStates), robotPose.getRotation());
        Angle turretPosition = m_turretPosSup.get();

        m_aimTarget = calculateTarget(robotPose);
        m_shotCalcOutputs = calcShot(robotPose, m_useSotm, m_aimTarget, turretPosition, robotChassisSpeeds);

        // Logging
        log_globalShotTarget.accept(m_aimTarget);

        log_loopTime.accept(m_calcTimer.get() * 1000.0);
    }

    /**
     * Sets the target to a Pose on the field relative to where the robot is.
     * EX: Robot in alliance zone red -> Red Hub Center
     * Executes Passing and Shooting aiming.
     * 
     * @param robotPose where the robot currently is
     * @return target pose
     */
    private Translation3d calculateTarget(Pose2d robotPose) {
        // m_currentTarget = AllianceFlipUtil.apply(target);
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation3d theTarget = FieldConstants.Hub.blueInnerCenterPoint;

        boolean robotPastOurZoneX = isRed ? robotPose.getMeasureX().lt(AllianceZoneUtil.redHubCenter.getMeasureX())
                : robotPose.getMeasureX().gt(AllianceZoneUtil.blueHubCenter.getMeasureX());

        if (robotPastOurZoneX) {
            boolean robotLeftOfCenter = isRed ? robotPose.getMeasureY().lt(AllianceZoneUtil.centerField_y_pos)
                    : robotPose.getMeasureY().gt(AllianceZoneUtil.centerField_y_pos);
            theTarget = robotLeftOfCenter ? ShooterK.kPassingSpotLeft : ShooterK.kPassingSpotRight;
        }
        return AllianceFlipUtil.apply(theTarget);
    }

    public record AzimuthCalcDetails(Angle turretReference, Pose3d desiredAimPose, Pose3d currentAimPose, double rawDesiredRotations, AngularVelocity turretVelocityFF) {}

    /**
     * Calculates the turret's *TARGET* angle while ensuring it stays within
     * physical limits.
     * IF the turret is near a limit, snaps 360 degrees in the opposite direction to
     * reach the same angle
     * without hitting the hardstop.
     * Note that if you have less than 360 degrees on the turret, you will simply
     * snap back to the other hard limit.
     * 
     * @param target target position
     * @return safe rotation setpoint that is accurate to the target within bounds
     *         of kTurretMaxAngle
     *         and kTurretMinAngle
     */
    public static AzimuthCalcDetails calcAzimuth(Translation3d target, Pose2d robotPose, Angle turretPosition, ChassisSpeeds fieldSpeeds) {
        // Convert once; reused below in both snapback and current-aim logging
        double turretPositionRots = turretPosition.in(Rotations);

        /* Calculation Zone */
        // turret pivot location in field space (no extra rotateBy — that's for
        // visualization only)
        Pose3d turretPose = new Pose3d(robotPose).transformBy(kTurretTransform);
        Translation3d turretTranslation = turretPose.getTranslation();

        // vector from turret pivot to target in field space
        Translation3d distance = target.minus(turretTranslation);

        // field-frame yaw to target, converted to turret-relative by subtracting
        // turret's zero direction
        // kTurretYawOffsetRad = robot heading + kTurretAngleOffset, so this correctly
        // accounts for
        // the physical offset of the turret's zero position relative to the robot's
        // forward direction
        double fieldYawRad = Math.atan2(distance.getY(), distance.getX());
        Rotation2d turretZeroFieldDir = turretPose.getRotation().toRotation2d();

        // Avoid Rotation2d allocation — subtract in radians and convert to rotations
        // directly
        Rotation2d direction = new Rotation2d(fieldYawRad).minus(turretZeroFieldDir);

        // desired aim: turret pivot with X-axis pointing at target in field space
        var desiredAimPose = new Pose3d(turretTranslation, new Rotation3d(0, 0, fieldYawRad));
        // current aim: turret pivot with X-axis showing where the turret is actually
        // pointing right now
        double currentFieldYaw = turretZeroFieldDir.getRadians() + turretPositionRots * (2 * Math.PI);
        var currentAimPose = new Pose3d(turretTranslation, new Rotation3d(0, 0, currentFieldYaw));

        // normalizes the angle to be fit in the range of the max rotations
        double angleRotations = MathUtil.inputModulus(
                direction.getRotations(), kTurretMinRotsMagnitudeD, kTurretMaxRotsMagnitudeD);

        /* Snapback Zone */
        double snapbackSafeAngleRotations = angleRotations;
        // this is the snapback function, to make sure that you will always be tracking
        // and you will not go over your physical limits.
        if (turretPositionRots > 0 && angleRotations + 1 <= kTurretMaxRotsD) {
            snapbackSafeAngleRotations += 1;
        } else if (turretPositionRots < 0 && angleRotations - 1 >= kTurretMinRotsD) {
            snapbackSafeAngleRotations -= 1;
        }


        var turretReference = Rotations.of(snapbackSafeAngleRotations);

        double dx = distance.getX();
        double dy = distance.getY();
        double d2 = dx * dx + dy * dy;
        double turretFFRadPerSec = d2 > 0
            ? (dy * fieldSpeeds.vxMetersPerSecond - dx * fieldSpeeds.vyMetersPerSecond) / d2
                - fieldSpeeds.omegaRadiansPerSecond
            : 0.0;

        var calcDetails = new AzimuthCalcDetails(
            turretReference, desiredAimPose, currentAimPose, angleRotations, RotationsPerSecond.of(turretFFRadPerSec));
        // logger.accept(calcDetails);
        return calcDetails;
    }

    public final record ShotCalcOutputs(
        AzimuthCalcDetails turretCalcDetails,
        ShotData shotData,
        Angle turretReference,
        Angle hoodReference,
        AngularVelocity shooterReference
    ) {}

    /**
     * Calculates the ideal shot to put the FUEL™ into the HUB™
     * Accounts for moving speeds
     * 
     * @param robotPose current Robot position.
     */
    public static ShotCalcOutputs calcShot(
        Pose2d robotPose,
        boolean staticShot,
        Translation3d target,
        Angle turretPosition,
        ChassisSpeeds chassisSpeeds
    ) {
        // How fast the robot is currently going, (CURRENT ROBOT VELOCITY)
        ChassisSpeeds fieldSpeeds = staticShot ? WpiK.kZeroChassisSpeeds : chassisSpeeds;
        // The Calculated shot itself, according to the current robotPose, robotSpeeds,
        // and the currentTarget
        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
            robotPose, fieldSpeeds, target, 3);

        // The turret angle according to the Calculated shot
        var azCalcDetails = calcAzimuth(calculatedShot.getTarget(), robotPose, turretPosition, fieldSpeeds);

        var turretReference = azCalcDetails.turretReference();
        var hoodReference = Degrees.of(calculatedShot.getHoodAngle().in(Degrees));
        var shooterReference = ShotCalculator.linearToAngularVelocity(
            calculatedShot.getExitVelocity(), kFlywheelRadius);
        return new ShotCalcOutputs(
            azCalcDetails, calculatedShot, turretReference, hoodReference, shooterReference);
    }
}
