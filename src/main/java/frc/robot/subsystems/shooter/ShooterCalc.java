package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
    private final DoubleLogger log_loopTime = WaltLogger.logDouble("ShotCalc", "LoopTimeMsec");


    // Precomputed doubles for hot-path unit conversions
    private static final double kTurretMinRotsD = kTurretMinRots.in(Rotations);
    private static final double kTurretMinRotsMagnitudeD = kTurretMinRots.magnitude();
    private static final double kTurretMaxRotsD = kTurretMaxRots.in(Rotations);
    private static final double kTurretMaxRotsMagnitudeD = kTurretMaxRots.magnitude();

    private final ShotData kEmptyShotData = new ShotData(0, 0);
    private final AzimuthCalcDetails kEmptyAzimuthCalcDetails = new AzimuthCalcDetails(0, 0, 0);
    private final ShotCalcOutputs kEmptyShotCalcOutputs = new ShotCalcOutputs(kEmptyAzimuthCalcDetails, kEmptyShotData, 0, 0, 0);

    private boolean m_useStaticShot = true;
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
        Angle turretPosition = m_turretPosSup.get();

        m_aimTarget = calculateTarget(robotPose);
        m_shotCalcOutputs = calcShot(robotPose, m_useStaticShot, m_aimTarget, turretPosition, robotChassisSpeeds);

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

    /** All fields raw doubles in CTRE-native units (rotations, rot/s). */
    public record AzimuthCalcDetails(
        double turretReferenceRots,
        double rawDesiredRotations,
        double turretVelocityFFRotPerSec
    ) {}

    /**
     * Calculates the turret's *TARGET* angle while ensuring it stays within
     * physical limits.
     * IF the turret is near a limit, snaps 360 degrees in the opposite direction to
     * reach the same angle without hitting the hardstop.
     *
     * All internal math uses raw doubles; only wraps to measure types for the output record.
     */
    public static AzimuthCalcDetails calcAzimuth(Translation3d target, Pose2d robotPose, Angle turretPosition, ChassisSpeeds fieldSpeeds) {
        double turretPositionRots = turretPosition.in(Rotations);
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double headingRad = robotPose.getRotation().getRadians();

        // Turret pivot in field space (inline transform — no Pose3d allocation)
        double cosR = Math.cos(headingRad);
        double sinR = Math.sin(headingRad);
        double turretX = robotX + kTurretOffsetX_m * cosR - kTurretOffsetY_m * sinR;
        double turretY = robotY + kTurretOffsetX_m * sinR + kTurretOffsetY_m * cosR;

        // Vector from turret pivot to target
        double dx = target.getX() - turretX;
        double dy = target.getY() - turretY;

        // Field-frame yaw to target
        double fieldYawRad = Math.atan2(dy, dx);

        // Turret zero direction in field frame
        double turretZeroRad = headingRad + kTurretAngleOffsetRad;

        // Direction in turret frame (equivalent to Rotation2d subtraction)
        double dirRad = fieldYawRad - turretZeroRad;
        // Normalize to [-pi, pi] then convert to rotations
        dirRad = Math.atan2(Math.sin(dirRad), Math.cos(dirRad));
        double dirRots = dirRad / (2 * Math.PI);

        // Normalize to turret range
        double angleRotations = MathUtil.inputModulus(dirRots, kTurretMinRotsMagnitudeD, kTurretMaxRotsMagnitudeD);

        /* Snapback Zone */
        double snapbackSafeAngleRotations = angleRotations;
        if (turretPositionRots > 0 && angleRotations + 1 <= kTurretMaxRotsD) {
            snapbackSafeAngleRotations += 1;
        } else if (turretPositionRots < 0 && angleRotations - 1 >= kTurretMinRotsD) {
            snapbackSafeAngleRotations -= 1;
        }

        // Turret velocity feedforward (compute in rad/s, convert to rot/s for CTRE)
        double d2 = dx * dx + dy * dy;
        double turretFFRadPerSec = d2 > 0
            ? (dy * fieldSpeeds.vxMetersPerSecond - dx * fieldSpeeds.vyMetersPerSecond) / d2
                - fieldSpeeds.omegaRadiansPerSecond
            : 0.0;

        return new AzimuthCalcDetails(
            snapbackSafeAngleRotations,
            angleRotations,
            turretFFRadPerSec / (2.0 * Math.PI));
    }

    /** All fields raw doubles in CTRE-native units (rotations, rot/s). */
    public final record ShotCalcOutputs(
        AzimuthCalcDetails turretCalcDetails,
        ShotData shotData,
        double turretReferenceRots,
        double hoodReferenceRots,
        double shooterReferenceRotPerSec
    ) {}

    /**
     * Calculates the ideal shot to put the FUEL™ into the HUB™
     * Accounts for moving speeds.
     *
     * All internal math uses raw doubles; wraps to measure types at the output boundary.
     */
    public static ShotCalcOutputs calcShot(
        Pose2d robotPose,
        boolean staticShot,
        Translation3d target,
        Angle turretPosition,
        ChassisSpeeds chassisSpeeds
    ) {
        ChassisSpeeds fieldSpeeds = staticShot ? WpiK.kZeroChassisSpeeds : chassisSpeeds;

        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(
            robotPose, fieldSpeeds, target, 3);

        var azCalcDetails = calcAzimuth(calculatedShot.getTarget(), robotPose, turretPosition, fieldSpeeds);

        return new ShotCalcOutputs(
            azCalcDetails,
            calculatedShot,
            azCalcDetails.turretReferenceRots(),
            calculatedShot.hoodAngle() / (2.0 * Math.PI),
            calculatedShot.exitVelocity() / (2.0 * Math.PI));
    }
}
