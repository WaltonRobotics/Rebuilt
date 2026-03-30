package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
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
import frc.util.WaltLogger.Pose2dLogger;
import frc.util.WaltLogger.Pose3dLogger;
import frc.util.WaltLogger.Translation3dArrayLogger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterK.*;

public class ShooterCalc {
    private final Supplier<SwerveDriveState> m_threadsafeSwerveDriveStateSup;
    private final DoubleSupplier m_turretPosRotsSup;
    private final SwerveDriveKinematics m_swerveKinematics = new SwerveDriveKinematics(TunerConstants.moduleTranslations);

    private final Pose3dLogger log_globalShotTarget = WaltLogger.logPose3d("ShotCalc", "globalTarget");
    private final Pose3dLogger log_calculatedShotTarget = WaltLogger.logPose3d("ShotCalc", "shotCalcTarget");
    private final DoubleLogger log_rawDesiredTurretRot = WaltLogger.logDouble("ShotCalc", "rawDesiredTurretRots");
    private final DoubleLogger log_desiredTurretRot = new DoubleLogger("ShotCalc", "desiredTurretRotations");
    private final Pose3dLogger log_desiredAimPose = WaltLogger.logPose3d("ShotCalc", "DesiredAimPose");
    private final Pose3dLogger log_currentAimPose = WaltLogger.logPose3d("ShotCalc", "CurrentAimPose");
    private final Translation3dArrayLogger log_ballTrajectory = WaltLogger.logTranslation3dArray("ShotCalc", "ballTrajectory");
    private final DoubleLogger log_loopTime = WaltLogger.logDouble("ShotCalc", "LoopTimeMsec");
    private static final Pose3dLogger log_turretRobotPose = WaltLogger.logPose3d("ShotCalc", "turretRobotPose");
    private static final Pose3dLogger log_turretFieldPose = WaltLogger.logPose3d("ShotCalc", "turretFieldPose");
    // private static final Pose3dLogger log_turret


    // Precomputed doubles for calculateTarget zone checks
    private static final double kRedHubCenterX = AllianceZoneUtil.redHubCenter.getX();
    private static final double kBlueHubCenterX = AllianceZoneUtil.blueHubCenter.getX();
    private static final double kCenterFieldYM = AllianceZoneUtil.centerField_y_pos.baseUnitMagnitude();

    // Precomputed doubles for hot-path unit conversions
    private static final double kTurretMinRotsD = kTurretMinRots.in(Rotations);
    private static final double kTurretMinRotsMagnitudeD = kTurretMinRots.magnitude();
    private static final double kTurretMaxRotsD = kTurretMaxRots.in(Rotations);
    private static final double kTurretMaxRotsMagnitudeD = kTurretMaxRots.magnitude();

    private final ShotData kEmptyShotData = new ShotData(0, 0);
    private final AzimuthCalcDetails kEmptyAzimuthCalcDetails = new AzimuthCalcDetails(0, new Pose3d(), new Pose3d(), 0, 0);
    private final ShotCalcOutputs kEmptyShotCalcOutputs = new ShotCalcOutputs(kEmptyAzimuthCalcDetails, kEmptyShotData, 0, 0, 0);

    private volatile boolean m_useStaticShot = true;
    private volatile Translation3d m_aimTarget = Translation3d.kZero;
    private volatile ShotCalcOutputs m_shotCalcOutputs = kEmptyShotCalcOutputs;
    
    private final Notifier m_notifier = new Notifier(this::calcCallback);
    private final Timer m_calcTimer = new Timer();

    public ShooterCalc(Supplier<SwerveDriveState> threadsafeSwerveDriveStateSup, DoubleSupplier turretPosSup) {
        m_threadsafeSwerveDriveStateSup = threadsafeSwerveDriveStateSup;
        m_turretPosRotsSup = turretPosSup;

        m_notifier.setName("ShooterCalc");
        m_notifier.startPeriodic(Hertz.of(25)); // 2x slower than robot loop
    }

    public void shouldUseStaticShot(boolean should) {
        m_useStaticShot = should;
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
        double turretPositionRots = m_turretPosRotsSup.getAsDouble();

        m_aimTarget = calculateTarget(robotPose);
        m_shotCalcOutputs = calcShot(robotPose, m_useStaticShot, m_aimTarget, turretPositionRots, robotChassisSpeeds);

        // Logging
        log_globalShotTarget.accept(m_aimTarget);
        log_desiredAimPose.accept(m_shotCalcOutputs.turretCalcDetails().desiredAimPose());
        log_currentAimPose.accept(m_shotCalcOutputs.turretCalcDetails().currentAimPose());
        log_rawDesiredTurretRot.accept(m_shotCalcOutputs.turretCalcDetails().rawDesiredRotations());
        log_desiredTurretRot.accept(m_shotCalcOutputs.turretCalcDetails().turretReferenceRots());

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

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        boolean robotPastOurZoneX = isRed ? robotX < kRedHubCenterX : robotX > kBlueHubCenterX;

        if (robotPastOurZoneX) {
            Translation3d leftPassPoseShifted = new Translation3d(ShooterK.kPassingXAsDouble, MathUtil.clamp(FieldConstants.fieldWidth / 2 + robotY, FieldConstants.fieldWidth / 2 + 1, FieldConstants.fieldWidth - 1), 0);
            Translation3d rightPassPoseShifted = new Translation3d(ShooterK.kPassingXAsDouble, MathUtil.clamp(robotY - FieldConstants.fieldWidth / 2, 1, FieldConstants.fieldWidth / 2 - 1), 0);
            boolean robotLeftOfCenter = isRed ? robotY < kCenterFieldYM : robotY > kCenterFieldYM;
            theTarget = robotLeftOfCenter ? leftPassPoseShifted : rightPassPoseShifted;
        }
        log_ballTrajectory.accept(new Translation3d[]{new Translation3d(FieldConstants.fieldLength - robotPose.getX(), FieldConstants.fieldWidth - robotPose.getY(), 0), theTarget});
        return AllianceFlipUtil.apply(theTarget);
    }

    public record AzimuthCalcDetails(double turretReferenceRots, Pose3d desiredAimPose, Pose3d currentAimPose, double rawDesiredRotations, double turretVelocityFF) {}

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
    // Precomputed for calcAzimuth logging
    private static final double kTurretOffsetZ_m = kTurretTransform.getTranslation().getZ();
    private static final Rotation3d kTurretRealPoseRotation =
        new Rotation3d(0, 0, -(kTurretAngleOffset.plus(Rotation2d.kPi)).getRadians());

    public static AzimuthCalcDetails calcAzimuth(Translation3d target, Pose2d robotPose, double turretHeading, ChassisSpeeds fieldSpeeds) {
        // Compute turret pivot position and zero direction with raw doubles
        // (eliminates Pose3d(robotPose).transformBy() + Rotation2d allocations)
        double headingRad = robotPose.getRotation().getRadians();
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double turretX = robotX + kTurretOffsetX_m * cosH - kTurretOffsetY_m * sinH;
        double turretY = robotY + kTurretOffsetX_m * sinH + kTurretOffsetY_m * cosH;
        double turretZeroFieldDirRad = headingRad + kTurretAngleOffsetRad;

        // Build Translation3d once for logging Pose3d objects
        Translation3d turretTranslation = new Translation3d(turretX, turretY, kTurretOffsetZ_m);

        double turretHeadingRots = turretHeading;
        Pose3d turretRobotPose = new Pose3d(turretTranslation,
            new Rotation3d(0, 0, turretZeroFieldDirRad + turretHeadingRots * (2 * Math.PI)));
        log_turretRobotPose.accept(turretRobotPose);

        Pose3d turretRealPose = new Pose3d(turretTranslation, kTurretRealPoseRotation);
        log_turretFieldPose.accept(turretRealPose);

        // Vector from turret to target for yaw calculation
        double toTargetX = target.getX() - turretX;
        double toTargetY = target.getY() - turretY;
        double fieldYawRad = Math.atan2(toTargetY, toTargetX);

        // Direction in rotations: normalize to [-0.5, 0.5] first (matches Rotation2d.minus behavior),
        // then clamp to turret range
        double directionRots = MathUtil.inputModulus(
                (fieldYawRad - turretZeroFieldDirRad) / (2.0 * Math.PI), -0.5, 0.5);

        // Logging poses
        var desiredAimPose = new Pose3d(turretTranslation, new Rotation3d(0, 0, fieldYawRad));
        double currentFieldYaw = turretZeroFieldDirRad + turretHeadingRots * (2 * Math.PI);
        var currentAimPose = new Pose3d(turretTranslation, new Rotation3d(0, 0, currentFieldYaw));

        double angleRotations = MathUtil.inputModulus(
                directionRots, kTurretMinRotsMagnitudeD, kTurretMaxRotsMagnitudeD);

        /* Snapback Zone */
        double snapbackSafeAngleRotations = angleRotations;
        // this is the snapback function, to make sure that you will always be tracking
        // and you will not go over your physical limits.
        if (turretHeadingRots > 0 && angleRotations + 1 <= kTurretMaxRotsD) {
            snapbackSafeAngleRotations += 1;
        } else if (turretHeadingRots < 0 && angleRotations - 1 >= kTurretMinRotsD) {
            snapbackSafeAngleRotations -= 1;
        }


        double turretReferenceRots = snapbackSafeAngleRotations;

        double d2 = toTargetX * toTargetX + toTargetY * toTargetY;
        double turretFFRadPerSec = d2 > 0
            ? (toTargetY * fieldSpeeds.vxMetersPerSecond - toTargetX * fieldSpeeds.vyMetersPerSecond) / d2
                - fieldSpeeds.omegaRadiansPerSecond
            : 0.0;

        AzimuthCalcDetails calcDetails = new AzimuthCalcDetails(
            turretReferenceRots, desiredAimPose, currentAimPose, angleRotations, turretFFRadPerSec);
        // logger.accept(calcDetails);
        return calcDetails;
    }

    public final record ShotCalcOutputs(
        AzimuthCalcDetails turretCalcDetails,
        ShotData shotData,
        double turretReferenceRots,
        double hoodReferenceRots,
        double shooterReferenceRps
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
        double turretPositionRots,
        ChassisSpeeds chassisSpeeds
    ) {
        // How fast the robot is currently going, (CURRENT ROBOT VELOCITY)
        // double speedMps = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        ChassisSpeeds fieldSpeeds = (staticShot /*|| speedMps < 0.1*/) ? WpiK.kZeroChassisSpeeds : chassisSpeeds;
        // The Calculated shot itself, according to the current robotPose, robotSpeeds,
        // and the currentTarget
        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
            robotPose, fieldSpeeds, target, 3);

        // The turret angle according to the Calculated shot
        AzimuthCalcDetails azCalcDetails = calcAzimuth(calculatedShot.getTarget(), robotPose, turretPositionRots, fieldSpeeds);

        double turretReferenceRots = azCalcDetails.turretReferenceRots();
        double hoodReferenceRots = calculatedShot.hoodAngle() / (2.0 * Math.PI);
        double shooterReferenceRPS = calculatedShot.exitVelocity() / (2.0 * Math.PI);
        return new ShotCalcOutputs(
            azCalcDetails, calculatedShot, turretReferenceRots, hoodReferenceRots, shooterReferenceRPS);
    }
}
