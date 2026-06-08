package frc.robot.subsystems.shooter.calc;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.WaltDriverStation;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.WpiK;
import frc.robot.generated.TunerConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.calc.ShotCalculator.ShotDataLerp;
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

/*
 * ShotCalcMath
 *
 * All the shot math runs here, on a background thread via WPILib's Notifier,
 * so it never eats into the main 50 Hz robot loop. The thread runs at 75 Hz
 * and drops the result into a volatile field. Shooter.periodic() just reads
 * that field every loop, no locking needed.
 *
 * Quick vocabulary:
 *   SOTM (Shoot On The Move) - the iterative algorithm that compensates for
 *        the robot moving while the ball is in the air. If we're driving right
 *        at 1 m/s and the ball takes 0.3 s to reach the hub, we aim a bit left.
 *   Volatile - Java keyword that tells the JVM other threads may write this.
 *        Without it, the main thread might see a stale cached value.
 *   Snapback - when the turret is near a hardstop and the target wraps to the
 *        other side, we add +-1 full rotation to the setpoint so the turret goes
 *        the short way around instead of slamming into the limit.
 *
 * One important rule: Shooter.periodic() caches the turret position into a
 * volatile double at the top of every loop. ShotCalcMath reads it from the
 * background thread. Don't move that line or the thread safety breaks.
 */
public class ShotCalcMath {
    private static final String kLogTab = "ShotCalc";

    // Tunable lateral bias correction. Balls curve left/right depending on which way
    // the turret faces (sinusoidal - no correction straight ahead, max at 90 degrees).
    // Enable and tune via /ShotCalc/lateralBiasGainRots in NetworkTables.
    private static final WaltTunable kLateralBiasTuner =
        new WaltTunable("/ShotCalc/lateralBiasGainRots", kTurretLateralBiasGainRots);

    // ---- INPUTS ----
    // Read every callback cycle. SwerveDriveState comes through a thread-safe supplier
    // from CTRE's odometry thread. Turret position is written by Shooter.periodic() into
    // a volatile field and read here.

    private final Supplier<SwerveDriveState> m_threadsafeSwerveDriveStateSup;
    private final DoubleSupplier m_turretPosRotsSup;
    private final SwerveDriveKinematics m_swerveKinematics = new SwerveDriveKinematics(TunerConstants.moduleTranslations);

    // ---- LOGGERS ----

    private final Pose3dLogger log_globalShotTarget = WaltLogger.logPose3d(kLogTab, "globalTarget");
    // private final Pose3dLogger log_calculatedShotTarget = WaltLogger.logPose3d(kLogTab, "shotCalcTarget");
    private final DoubleLogger log_rawDesiredTurretRot = WaltLogger.logDouble(kLogTab, "rawDesiredTurretRots");
    private final DoubleLogger log_desiredTurretRot = new DoubleLogger(kLogTab, "desiredTurretRotations");
    private final DoubleLogger log_timeOfFlight = new DoubleLogger(kLogTab, "timeOfFlight");
    private final Pose3dLogger log_desiredAimPose = WaltLogger.logPose3d(kLogTab, "DesiredAimPose");
    private final Pose3dLogger log_currentAimPose = WaltLogger.logPose3d(kLogTab, "CurrentAimPose");
    private final Translation3dArrayLogger log_ballTrajectory = WaltLogger.logTranslation3dArray(kLogTab, "ballTrajectory");
    private final DoubleLogger log_loopTime = WaltLogger.logDouble(kLogTab, "LoopTimeMsec");
    // private static final Pose3dLogger log_turretRobotPose = WaltLogger.logPose3d(kLogTab, "turretRobotPose");
    // private static final Pose3dLogger log_turretFieldPose = WaltLogger.logPose3d(kLogTab, "turretFieldPose");
    private final BooleanLogger log_robotPastOurZoneX = WaltLogger.logBoolean(kLogTab, "robotInOurZone");
    private final BooleanLogger log_robotInHubPassingZone = WaltLogger.logBoolean(kLogTab, "robotInHubPassingZone");

    // ---- PRECOMPUTED CONSTANTS ----
    // Calculated once at startup and reused every callback to skip redundant lookups.

    // Hub X positions for the zone-boundary check in calculateTarget()
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

    // Pre-allocated log buffer so we don't create a new Translation3d[] every callback
    private static final Translation3d[] m_ballTrajBuffer = new Translation3d[2];

    // Turret limits as raw doubles to skip .in(Rotations) calls on the hot path
    private static final double kTurretMinRotsD = kTurretMinRots.in(Rotations);
    private static final double kTurretMinRotsMagnitudeD = kTurretMinRots.magnitude();
    private static final double kTurretMaxRotsD = kTurretMaxRots.in(Rotations);
    private static final double kTurretMaxRotsMagnitudeD = kTurretMaxRots.magnitude();

    // ---- VOLATILE STATE ----
    // Written by the Notifier thread, read by the main thread.
    // volatile means the main thread always reads the latest write, no lock needed.

    private volatile boolean m_useStaticShot = true;
    private static volatile Translation3d m_aimTarget = Translation3d.kZero;
    private volatile ShotCalcOutputs m_shotCalcOutputs;
    private static volatile boolean m_isPassingFlag = false; // true when robot is past the hub
    private static final BooleanSupplier m_isPassing = () -> m_isPassingFlag;

    // ---- THREAD + TIMER ----

    private final Notifier m_notifier = new Notifier(this::calcCallback);
    private final Timer m_calcTimer = new Timer(); // how long each callback takes

    // ---- RUNTIME STATE ----

    private boolean isRed = WaltDriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    private boolean robotInNoPassingZone; // true when near opponent hub in the no-pass zone


    // =============================================================
    // CONSTRUCTOR
    // =============================================================

    public ShotCalcMath(Supplier<SwerveDriveState> threadsafeSwerveDriveStateSup, DoubleSupplier turretPosSup) {
        m_threadsafeSwerveDriveStateSup = threadsafeSwerveDriveStateSup;
        m_turretPosRotsSup = turretPosSup;

        // Initialize with empty values so Shooter.periodic() never gets null back
        ShotDataLerp emptyShotData = new ShotDataLerp(0.0, 0.0, new Translation3d(), 0.0);
        AzimuthCalcDetails emptyAzimuth = new AzimuthCalcDetails(0, 0, 0, 0, 0, 0, 0);
        m_shotCalcOutputs = new ShotCalcOutputs(emptyAzimuth, emptyShotData, 0, 0, 0);

        m_notifier.setName("ShotCalcMath");
        m_notifier.startPeriodic(Hertz.of(75)); // 2x slower than robot loop
    }


    // =============================================================
    // PUBLIC API
    // =============================================================

    public void shouldUseStaticShot(boolean should) {
        m_useStaticShot = should;
    }

    public static Translation3d getLatestAimTarget() {
        return m_aimTarget;
    }

    // Called by Shooter.periodic() every loop to grab the latest shot parameters.
    // Reading one volatile reference is about as cheap as it gets.
    public ShotCalcOutputs getLatestShotCalcOutputs() {
        return m_shotCalcOutputs;
    }

    // ShotCalculator reads this to pick kShotTable vs kPassingTable without
    // needing to hold a reference back to this class.
    public static BooleanSupplier isPassing() {
        return m_isPassing;
    }


    // =============================================================
    // BACKGROUND THREAD LOOP
    // =============================================================

    // Runs at 75 Hz on the Notifier's background thread.
    // calculateTarget() must go first because it sets m_isPassingFlag,
    // which calcShot() reads through ShotCalculator.isPassing() to pick the right LERP table.
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

        var details = m_shotCalcOutputs.turretCalcDetails();
        Translation3d turretTranslation = new Translation3d(details.turretX(), details.turretY(), kTurretOffsetZ_m);
        log_desiredAimPose.accept(new Pose3d(turretTranslation, new Rotation3d(0, 0, details.fieldYawRad())));
        log_currentAimPose.accept(new Pose3d(turretTranslation, new Rotation3d(0, 0, details.currentFieldYawRad())));
        log_rawDesiredTurretRot.accept(details.rawDesiredRotations());
        log_desiredTurretRot.accept(details.turretReferenceRots());
        log_timeOfFlight.accept(m_shotCalcOutputs.shotData().tofSec());

        log_loopTime.accept(m_calcTimer.get() * 1000.0);
    }


    // =============================================================
    // TARGET SELECTION
    // =============================================================

    // Picks which field target to aim at based on where the robot is.
    //
    // Robot on own side of hub -> shoot at the enemy hub center
    // Robot past the hub into neutral zone -> pass to a passing point
    //   about where the bump is, left or right depending on which side of field center
    //
    // m_isPassingFlag is set here and read by ShotCalculator to pick kPassingTable.
    // robotInNoPassingZone tracks when passing angles near the opponent hub are unsafe.

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

    // private boolean underTrench(Pose2d turretPose) {
    //     isRed = WaltDriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    //     double robotX = turretPose.getX();
    //     double robotY = turretPose.getY();

    //     boolean inTrenchZone = isRed
    //         ? (robotY < FieldConstants.LinesHorizontal.rightTrenchOpenStart || robotY > FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
    //         : (robotY > FieldConstants.LinesHorizontal.rightTrenchOpenStart || robotY < FieldConstants.LinesHorizontal.leftTrenchOpenEnd);

    //     double trenchCenterX = isRed
    //         ? FieldConstants.LinesVertical.oppHubCenter
    //         : FieldConstants.LinesVertical.hubCenter;
    //     double trenchHalfDepth = FieldConstants.LeftTrench.depth / 2.0;
    //     boolean inTrenchX = Math.abs(robotX - trenchCenterX) < trenchHalfDepth + 0.3; // 0.3m buffer

    //     log_inTrenchZone.accept(inTrenchZone);

    //     log_underTrench.accept(inTrenchZone && inTrenchX);

    //     if (inTrenchZone && inTrenchX) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }


    // =============================================================
    // SHOT + AZIMUTH CALCULATION
    // =============================================================

    // Precomputed for calcAzimuth — skip calling kTurretTransform.getTranslation().getZ() every cycle
    private static final double kTurretOffsetZ_m = kTurretTransform.getTranslation().getZ();
    // private static final Rotation3d kTurretRealPoseRotation =
    //     new Rotation3d(0, 0, -(kTurretAngleOffset.plus(Rotation2d.kPi)).getRadians());

    
    /**
     * 
     * Computes the turret setpoint in rotations and velocity feedforward.
     * Uses raw doubles to avoid Pose3d/Rotation2d allocations on the hot path.
     * 
     * What it does:
     * - Computes the turret pivot position in field coords from robot heading + physical offset
     * - atan2 from pivot to target gives the desired field yaw
     * - Subtracts turret zero direction, normalizes to [-0.5, 0.5] rotations
     * - Applies sinusoidal lateral bias correction if enabled
     * - Snapback: if near a soft limit, shifts the setpoint +-1 rotation so the
     *     turret wraps the short way around instead of crossing the hardstop
     * - Velocity FF from tangential velocity of the target relative to the pivot
     *
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

        //vx | vy is turret pivot not absolute field position
        double vx = fieldSpeeds.vxMetersPerSecond - (turretY - robotY) * fieldSpeeds.omegaRadiansPerSecond; //-ry * omega
        double vy = fieldSpeeds.vyMetersPerSecond + (turretX - robotX) * fieldSpeeds.omegaRadiansPerSecond; //+rx * omega

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

        // double d2 = toTargetX * toTargetX + toTargetY * toTargetY;
        // double turretFFRadPerSec = d2 > 0
        //     ? (toTargetY * fieldSpeeds.vxMetersPerSecond - toTargetX * fieldSpeeds.vyMetersPerSecond) / d2
        //         - fieldSpeeds.omegaRadiansPerSecond
        //     : 0.0;


        double distance = Math.hypot(toTargetX, toTargetY);
        double tangentialVel = (toTargetX * vx - toTargetY * vy) / distance;
        double turretFFRadPerSec = tangentialVel / distance;

        turretFFRadPerSec -= fieldSpeeds.omegaRadiansPerSecond;

        AzimuthCalcDetails calcDetails = new AzimuthCalcDetails(
            turretReferenceRots, turretFFRadPerSec,
            turretX, turretY,
            fieldYawRad, currentFieldYawRad,
            angleRotations);
        calcDetails.acceptLogging(calcDetails);
        return calcDetails;
    }

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
        ShotCalcOutputs outputs = new ShotCalcOutputs(azCalcDetails, calculatedShot, turretReferenceRots, hoodReferenceRots, shooterReferenceRPS);
        outputs.acceptLogging(outputs);
        return outputs;
    }


    // =============================================================
    // OUTPUT RECORDS
    // =============================================================

    /*
     * AzimuthCalcDetails
     *
     * Everything calcAzimuth() knows about where the turret should point.
     * This gets passed up through ShotCalcOutputs to Shooter.periodic().
     *
     *   turretReferenceRots - final snapback-safe setpoint in rotations
     *   turretVelocityFF    - angular velocity feedforward in rad/s
     *   turretX / turretY   - turret pivot in field coords (for logging)
     *   fieldYawRad         - desired absolute yaw in radians
     *   currentFieldYawRad  - current turret yaw in field coords (shows tracking lag)
     *   rawDesiredRotations - pre-snapback angle, useful for debugging wrap-around
     */
    public record AzimuthCalcDetails(
        double turretReferenceRots, double turretVelocityFF,
        double turretX, double turretY,
        double fieldYawRad, double currentFieldYawRad,
        double rawDesiredRotations
        ) {
        private static final String kCalcTab = "/AzimuthCalcDetails";

        private static final DoubleLogger log_turretReferenceRots = new DoubleLogger(kLogTab + kCalcTab, "turretReferenceRots");
        private static final DoubleLogger log_turretVelocityFF = new DoubleLogger(kLogTab + kCalcTab, "turretVelocityFF");
        private static final DoubleLogger log_turretX = new DoubleLogger(kLogTab + kCalcTab, "turretX");
        private static final DoubleLogger log_turretY = new DoubleLogger(kLogTab + kCalcTab, "turretY");
        private static final DoubleLogger log_fieldYawRad = new DoubleLogger(kLogTab + kCalcTab, "fieldYawRad");
        private static final DoubleLogger log_currentFieldYawRad = new DoubleLogger(kLogTab + kCalcTab, "currentFieldYawRad");
        private static final DoubleLogger log_rawDesiredRotations = new DoubleLogger(kLogTab + kCalcTab, "rawDesiredRotations");

        public void acceptLogging(AzimuthCalcDetails details) {
            log_turretReferenceRots.accept(details.turretReferenceRots);
            log_turretVelocityFF.accept(details.turretVelocityFF);
            log_turretX.accept(details.turretX);
            log_turretY.accept(details.turretY);
            log_fieldYawRad.accept(details.fieldYawRad);
            log_currentFieldYawRad.accept(details.currentFieldYawRad);
            log_rawDesiredRotations.accept(details.rawDesiredRotations);
        }
    }

    /*
     * ShotCalcOutputs
     *
     * The full package of parameters Shooter.periodic() needs to drive the mechanism.
     * Produced once per 75 Hz callback and stored in a volatile field.
     *
     *   turretCalcDetails   - turret angle + feedforward (from calcAzimuth)
     *   shotData            - flywheel speed, hood angle, TOF, predicted target (from ShotCalculator)
     *   turretReferenceRots - pulled from turretCalcDetails for convenience
     *   hoodReferenceRots   - hood angle converted from radians to rotations
     *   shooterReferenceRps - flywheel speed converted from rad/s to RPS
     */
    public final record ShotCalcOutputs(
        AzimuthCalcDetails turretCalcDetails,
        ShotDataLerp shotData,
        double turretReferenceRots,
        double hoodReferenceRots,
        double shooterReferenceRps
    ) {
        private static final String kCalcTab = "/ShotCalcOutputs";

        private static final DoubleLogger log_turretReferenceRots = new DoubleLogger(kLogTab + kCalcTab, "turretReferenceRots");
        private static final DoubleLogger log_hoodReferenceRots = new DoubleLogger(kLogTab + kCalcTab, "hoodReferenceRots");
        private static final DoubleLogger log_shooterReferenceRPS = new DoubleLogger(kLogTab + kCalcTab, "shooterReferenceRPS");

        public void acceptLogging(ShotCalcOutputs outputs) {
            log_turretReferenceRots.accept(outputs.turretReferenceRots);
            log_hoodReferenceRots.accept(outputs.hoodReferenceRots);
            log_shooterReferenceRPS.accept(outputs.shooterReferenceRps);
        }
    }
}
