package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.WaltDriverStation;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.WpiK;
import frc.robot.generated.TunerConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShotCalculator.ShotDataLerp;
import frc.util.AllianceFlipUtil;
import frc.util.AllianceZoneUtil;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose3dLogger;
import frc.util.WaltLogger.Translation3dArrayLogger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterK.*;

import frc.util.WaltTunable;

public class ShooterCalc {
    private static final String kLogTab = "ShotCalc";
    private static final WaltTunable kLateralBiasTuner =
        new WaltTunable("/ShotCalc/lateralBiasGainRots", kTurretLateralBiasGainRots);

    private final Supplier<SwerveDriveState> m_threadsafeSwerveDriveStateSup;
    private final DoubleSupplier m_turretPosRotsSup;
    private final SwerveDriveKinematics m_swerveKinematics = new SwerveDriveKinematics(TunerConstants.moduleTranslations);

    private final BooleanTopic bt_canTurretShoot = new BooleanTopic(NetworkTableInstance.getDefault().getTopic(("ShooterCalc/isTurretAngry")));
    private final BooleanPublisher pub_canTurretShoot;

    private final Pose3dLogger log_globalShotTarget = WaltLogger.logPose3d(kLogTab, "globalTarget");
    // private final Pose3dLogger log_calculatedShotTarget = WaltLogger.logPose3d(kLogTab, "shotCalcTarget");
    private final DoubleLogger log_rawDesiredTurretRot = WaltLogger.logDouble(kLogTab, "rawDesiredTurretRots");
    private final DoubleLogger log_desiredTurretRot = new DoubleLogger(kLogTab, "desiredTurretRotations");
    private final DoubleLogger log_timeOfFlight = new DoubleLogger(kLogTab, "timeOfFlight");
    private final Pose3dLogger log_desiredAimPose = WaltLogger.logPose3d(kLogTab, "DesiredAimPose");
    private final Pose3dLogger log_currentAimPose = WaltLogger.logPose3d(kLogTab, "CurrentAimPose");
    private final Translation3dArrayLogger log_ballTrajectory = WaltLogger.logTranslation3dArray(kLogTab, "ballTrajectory");
    private final DoubleLogger log_loopTime = WaltLogger.logDouble(kLogTab, "LoopTimeMsec");
    private static final Pose3dLogger log_turretRobotPose = WaltLogger.logPose3d(kLogTab, "turretRobotPose");
    private static final Pose3dLogger log_turretFieldPose = WaltLogger.logPose3d(kLogTab, "turretFieldPose");
    private final BooleanLogger log_robotPastOurZoneX = WaltLogger.logBoolean(kLogTab, "robotInOurZone");
    private final BooleanLogger log_robotInHubPassingZone = WaltLogger.logBoolean(kLogTab, "robotInHubPassingZone");
    private final BooleanLogger log_canTurretShoot = WaltLogger.logBoolean(kLogTab, "canTurretShoot");
    private final BooleanLogger log_inTrenchZone = WaltLogger.logBoolean(kLogTab, "inTrenchZone");
    private final BooleanLogger log_underTrench = WaltLogger.logBoolean(kLogTab, "underTrench");

    // Precomputed doubles for calculateTarget zone checks
    private static final double kRedHubCenterX = AllianceZoneUtil.redHubCenter.getX();
    private static final double kBlueHubCenterX = AllianceZoneUtil.blueHubCenter.getX();
    private static final double kCenterFieldYM = AllianceZoneUtil.centerField_y_pos.baseUnitMagnitude();

    // Precomputed passing targets — values are constants so no need to allocate each callback
    private static final Translation3d kLeftPassTarget =
        new Translation3d(ShooterK.kPassingXAsDouble, FieldConstants.fieldWidth - 2, 0);
    private static final Translation3d kRightPassTarget =
        new Translation3d(ShooterK.kPassingXAsDouble, 2, 0);
    
    // private static final Translation3d kLeftPassPastHubTarget =
    //     new Translation3d(ShooterK.kPassingXAsDouble + 0.5, FieldConstants.fieldWidth - 1.5, 0);
    // private static final Translation3d kRightPassPastHubTarget =
    //     new Translation3d(ShooterK.kPassingXAsDouble + 0.5, 1.5, 0);

    // Pre-allocated ballTrajectory log buffer — reused each callback to avoid array allocation
    private static final Translation3d[] m_ballTrajBuffer = new Translation3d[2];

    // Precomputed doubles for hot-path unit conversions
    private static final double kTurretMinRotsD = kTurretMinRots.in(Rotations);
    private static final double kTurretMinRotsMagnitudeD = kTurretMinRots.magnitude();
    private static final double kTurretMaxRotsD = kTurretMaxRots.in(Rotations);
    private static final double kTurretMaxRotsMagnitudeD = kTurretMaxRots.magnitude();

    private final ShotDataLerp kEmptyShotData = new ShotDataLerp(0.0, 0.0, new Translation3d(), 0.0);
    private final AzimuthCalcDetails kEmptyAzimuthCalcDetails = new AzimuthCalcDetails(0, 0, 0, 0, 0, 0, 0);
    private final ShotCalcOutputs kEmptyShotCalcOutputs = new ShotCalcOutputs(kEmptyAzimuthCalcDetails, kEmptyShotData, 0, 0, 0);

    private volatile boolean m_useStaticShot = true;
    private static volatile Translation3d m_aimTarget = Translation3d.kZero;
    private volatile ShotCalcOutputs m_shotCalcOutputs = kEmptyShotCalcOutputs;
    private static volatile boolean m_isPassingFlag = false;
    private static final BooleanSupplier m_isPassing = () -> m_isPassingFlag;
    private static volatile boolean m_canTurretShoot = false;
    private static volatile boolean m_underTrench = false;

    private final Notifier m_notifier = new Notifier(this::calcCallback);
    private final Timer m_calcTimer = new Timer();

    private boolean isRed = WaltDriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    // private double robotX;
    // private double robotY;
    private boolean robotInNoPassingZone;

    public ShooterCalc(Supplier<SwerveDriveState> threadsafeSwerveDriveStateSup, DoubleSupplier turretPosSup) {
        m_threadsafeSwerveDriveStateSup = threadsafeSwerveDriveStateSup;
        m_turretPosRotsSup = turretPosSup;

        pub_canTurretShoot = bt_canTurretShoot.publish();
        m_notifier.setName("ShooterCalc");
        m_notifier.startPeriodic(Hertz.of(25)); // 2x slower than robot loop
    }

    public void shouldUseStaticShot(boolean should) {
        m_useStaticShot = should;
    }

    public static Translation3d getLatestAimTarget() {
        return m_aimTarget;
    }

    public ShotCalcOutputs getLatestShotCalcOutputs() {
        return m_shotCalcOutputs;
    }

    public static BooleanSupplier isPassing() {
        return m_isPassing;
    }

    public static boolean canTurretShoot() {
        return m_canTurretShoot;
    }

    public static boolean getUnderTrench() {
        return m_underTrench;
    }

    /**
     * first checks if we're passing. If we're not passing, then we can shoot whenever
     * If we are passing, it checks if we're NOT in the unable-to-pass range. If we're not in it, then the turret can shoot!
     * 
     * lowk should be checking if we are IN THE RANGE rather than greater than the outsides, but this is cope for now cuz sadness
     */
    private void refreshCanTurretShoot() {
        if (isPassing().getAsBoolean()) {
            if (m_turretPosRotsSup.getAsDouble() > kTurretMinNotAbleToPassRange && m_turretPosRotsSup.getAsDouble() < kTurretMaxNotAbleToPassRange/* && !robotInNoPassingZone */) {
                m_canTurretShoot = true;
            } else {
                m_canTurretShoot = false;
            }
            log_canTurretShoot.accept(m_canTurretShoot);
        } else {
            m_canTurretShoot = true;    //the turret can shoot anytime we are not passing
            log_canTurretShoot.accept(m_canTurretShoot);
        }
    }


    private void calcCallback() {
        m_calcTimer.restart();
        // getters from outside
        SwerveDriveState swerveState = m_threadsafeSwerveDriveStateSup.get();
        Pose2d robotPose = swerveState.Pose;
        ChassisSpeeds robotChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_swerveKinematics.toChassisSpeeds(swerveState.ModuleStates), robotPose.getRotation());
        double turretPositionRots = m_turretPosRotsSup.getAsDouble();
        Pose3d turretPose = new Pose3d(robotPose).transformBy(kTurretTransform);

        m_underTrench = underTrench(turretPose.toPose2d());
        m_aimTarget = calculateTarget(robotPose);
        m_shotCalcOutputs = calcShot(robotPose, m_useStaticShot, m_aimTarget, turretPositionRots, robotChassisSpeeds);
        refreshCanTurretShoot();

        // Logging
        pub_canTurretShoot.accept(canTurretShoot());
        log_globalShotTarget.accept(m_aimTarget);

        var details = m_shotCalcOutputs.turretCalcDetails();
        Translation3d turretTranslation = new Translation3d(details.turretX(), details.turretY(), kTurretOffsetZ_m);
        log_desiredAimPose.accept(new Pose3d(turretTranslation, new Rotation3d(0, 0, details.fieldYawRad())));
        log_currentAimPose.accept(new Pose3d(turretTranslation, new Rotation3d(0, 0, details.currentFieldYawRad())));
        log_rawDesiredTurretRot.accept(details.rawDesiredRotations());
        log_desiredTurretRot.accept(details.turretReferenceRots());
        log_timeOfFlight.accept(m_shotCalcOutputs.shotData().tofSec());

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
        isRed = WaltDriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation3d theTarget = FieldConstants.Hub.blueInnerCenterPoint;

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        boolean robotPastOurZoneX = isRed ? robotX < kRedHubCenterX : robotX > kBlueHubCenterX;
        log_robotPastOurZoneX.accept(robotPastOurZoneX);

        robotInNoPassingZone = (robotPastOurZoneX && (robotY < kNoPassZoneLeftY) && (robotY > kNoPassZoneRightY) && isRed)
            ? (robotX > FieldConstants.fieldLength - kNoPassZoneTopX)
            : robotX < kNoPassZoneTopX;
        log_robotInHubPassingZone.accept(robotInNoPassingZone);

        if (robotPastOurZoneX) {
            m_isPassingFlag = true;
            // if (!robotInNoPassingZone) {
            boolean robotLeftOfCenter = isRed ? robotY < kCenterFieldYM : robotY > kCenterFieldYM;
            theTarget = robotLeftOfCenter ? kLeftPassTarget : kRightPassTarget;
            // }
        } else {
            m_isPassingFlag = false;
        }
        m_ballTrajBuffer[0] = new Translation3d(FieldConstants.fieldLength - robotX, FieldConstants.fieldWidth - robotY, 0);
        m_ballTrajBuffer[1] = theTarget;
        log_ballTrajectory.accept(m_ballTrajBuffer);
        return AllianceFlipUtil.apply(theTarget);
    }

    //do we want to pass in the turret pose?
    private boolean underTrench(Pose2d turretPose) {
        isRed = WaltDriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        double robotX = turretPose.getX();
        double robotY = turretPose.getY();

        boolean inTrenchZone = isRed 
            ? (robotY < FieldConstants.LinesHorizontal.rightTrenchOpenStart || robotY > FieldConstants.LinesHorizontal.leftTrenchOpenEnd) 
            : (robotY > FieldConstants.LinesHorizontal.rightTrenchOpenStart || robotY < FieldConstants.LinesHorizontal.leftTrenchOpenEnd);

        double trenchCenterX = isRed 
            ? FieldConstants.LinesVertical.oppHubCenter 
            : FieldConstants.LinesVertical.hubCenter;
        double trenchHalfDepth = FieldConstants.LeftTrench.depth / 2.0;
        boolean inTrenchX = Math.abs(robotX - trenchCenterX) < trenchHalfDepth + 0.3; // 0.3m buffer

        log_inTrenchZone.accept(inTrenchZone);

        log_underTrench.accept(inTrenchZone && inTrenchX);

        if (inTrenchZone && inTrenchX) {
            return true;
        } else {
            return false;
        }
    }

    public record AzimuthCalcDetails(
        double turretReferenceRots, double turretVelocityFF,
        double turretX, double turretY,
        double fieldYawRad, double currentFieldYawRad,
        double rawDesiredRotations) {}

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

        double turretHeadingRots = turretHeading;

        // Vector from turret to target for yaw calculation
        double toTargetX = target.getX() - turretX;
        double toTargetY = target.getY() - turretY;
        double fieldYawRad = Math.atan2(toTargetY, toTargetX);

        // Direction in rotations: normalize to [-0.5, 0.5] first (matches Rotation2d.minus behavior),
        // then clamp to turret range
        double directionRots = MathUtil.inputModulus(
                (fieldYawRad - turretZeroFieldDirRad) / (2.0 * Math.PI), -0.5, 0.5);

        // Compensate for lateral ball bias that varies sinusoidally with turret angle relative to robot.
        // At 0/180° (fwd/back): no bias. At 90°: balls bias left, at 270°: bias right.
        // Enable via /ShotCalc/lateralBiasEnabled, tune gain via /ShotCalc/lateralBiasGainRots.
        if (kLateralBiasTuner.enabled()) {
            double turretRelToRobotRad = kTurretAngleOffsetRad + directionRots * 2.0 * Math.PI;
            directionRots -= kLateralBiasTuner.get() * Math.sin(turretRelToRobotRad);
        }

        double currentFieldYawRad = turretZeroFieldDirRad + turretHeadingRots * (2 * Math.PI);

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
            turretReferenceRots, turretFFRadPerSec,
            turretX, turretY,
            fieldYawRad, currentFieldYawRad,
            angleRotations);
        // logger.accept(calcDetails);
        return calcDetails;
    }

    public final record ShotCalcOutputs(
        AzimuthCalcDetails turretCalcDetails,
        ShotDataLerp shotData,
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
        ShotDataLerp calculatedShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
            robotPose, fieldSpeeds, target, 8);

        // The turret angle according to the Calculated shot
        AzimuthCalcDetails azCalcDetails = calcAzimuth(calculatedShot.getTarget(), robotPose, turretPositionRots, fieldSpeeds);

        double turretReferenceRots = azCalcDetails.turretReferenceRots();
        double hoodReferenceRots = calculatedShot.hoodAngle() / (2.0 * Math.PI);
        double shooterReferenceRPS = calculatedShot.exitVelocity() / (2.0 * Math.PI);
        return new ShotCalcOutputs(
            azCalcDetails, calculatedShot, turretReferenceRots, hoodReferenceRots, shooterReferenceRPS);
    }
}
