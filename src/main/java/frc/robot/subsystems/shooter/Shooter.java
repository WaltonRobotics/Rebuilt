
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.WpiK;
import frc.robot.subsystems.shooter.ShotCalculator.ShotData;
import frc.util.AllianceFlipUtil;
import frc.util.AllianceZoneUtil;
import frc.util.WaltMotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose3dLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    // boolean m_useShotCalculator = true;
    boolean m_holdTurretAtIntakePos = false;
    boolean m_turretLocked = false;
    Angle m_turretLockAngle = Degrees.zero();
    

    private AngularVelocity m_flywheelVelocity;
    // private Angle m_turretTurnPosition;

    private int m_fuelStored = 8;

    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;

    private boolean m_onLeftSide;
    // need to implement the differing targets (if in neutral zone, shoot to X point
    // (passing))
    private Translation3d m_currentTarget = Translation3d.kZero;

    private final TurretVisualizer m_turretVisualizer;
    private final FuelSim m_fuelSim;

    public final EventLoop homingEventLoop = new EventLoop();

    // ---MOTORS + CONTROL REQUESTS
    private final TalonFX m_shooterA = new TalonFX(kShooterA_CANID, Constants.kCanivoreBus); // X60Foc
    private final TalonFX m_shooterB = new TalonFX(kShooterB_CANID, Constants.kCanivoreBus); // X60Foc
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final NeutralOut m_neutralOutReq = new NeutralOut();
    

    private final TalonFX m_turret = new TalonFX(kTurretCANID, Constants.kCanivoreBus); // X44Foc
    private final CANcoder m_turretEncoderA = new CANcoder(19, Constants.kCanivoreBus);
    private final Canandmag m_turretEncoderB = new Canandmag(1);
    private final PositionVoltage m_PVRequest = new PositionVoltage(0).withEnableFOC(true);
    private final VoltageOut m_VoltageReq = new VoltageOut(0);
    private final StaticBrake m_BrakeReq = new StaticBrake();

    // private final TalonFXS m_hood = new TalonFXS(kHoodCANID);
    // private final PositionVoltage m_hoodPVRequest = new PositionVoltage(0).withEnableFOC(false);
    // private final VoltageOut m_hoodZeroReq = new VoltageOut(0);

    private final Hood m_hood = new Hood();

    private final DigitalInput m_turretHomingHall = new DigitalInput(2);
    private final Trigger trg_homingHallDirect = new Trigger(homingEventLoop, () -> !m_turretHomingHall.get());
    private final BooleanLogger log_turretHomingHall = new BooleanLogger(kLogTab, "turretHomeHall");

    private Angle m_calcTurret = Rotations.zero();
    private Supplier<AngularVelocity> m_calcFlywheelVelocity = () -> RotationsPerSecond.of(44.81);

    // Precomputed: true if turret travel is less than one full rotation (sub-360
    // cope path)
    private static final boolean kTurretSubRotation = kTurretMaxRotsFromHome.times(2).magnitude() < 1.0;
    // Precomputed doubles for hot-path unit conversions
    private static final double kTurretMinRotsD = kTurretMinRots.in(Rotations);
    private static final double kTurretMinRotsMagnitudeD = kTurretMinRots.magnitude();
    private static final double kTurretMaxRotsD = kTurretMaxRots.in(Rotations);
    private static final double kTurretMaxRotsMagnitudeD = kTurretMaxRots.magnitude();

    // Yaw offset of the turret's zero direction relative to the robot's forward
    // direction
    private static final double kTurretYawOffsetRad = kTurretTransform.getRotation().toRotation2d().getRadians();

    private final DoubleLogger log_calcFlywheelVelocity = new DoubleLogger("Shooter/Flywheel", "calcFlywheelVelocity");
    private final DoubleLogger log_calcTurretPos = new DoubleLogger("Shooter/Turret", "calcTurretPos");

    private final DoubleLogger log_rawDesiredTurretRot = WaltLogger.logDouble("ShotCalc", "rawDesiredTurretRots");
    private final DoubleLogger log_desiredTurretRot = new DoubleLogger("ShotCalc", "desiredTurretRotations");
    private final Pose3dLogger log_globalShotTarget = WaltLogger.logPose3d("ShotCalc", "globalTarget");
    private final Pose3dLogger log_calculatedShotTarget = WaltLogger.logPose3d("ShotCalc", "shotCalcTarget");

    // private static final DoubleLogger log_timeOfFlight = new
    // DoubleLogger("Shooter/Calculator", "timeOfFlight");

    private final Pose3dLogger log_desiredAimPose = WaltLogger.logPose3d("ShotCalc", "DesiredAimPose");
    private final Pose3dLogger log_currentAimPose = WaltLogger.logPose3d("ShotCalc", "CurrentAimPose");

    private final BooleanLogger log_turretHomed = WaltLogger.logBoolean("Shooter/Turret", "Homed");
    // private final BooleanLogger log_hoodHomed = WaltLogger.logBoolean("Shooter/Hood", "Homed");
    private final BooleanLogger log_useShotCalc = WaltLogger.logBoolean("ShotCalc", "useShotCalc");

    // ---LOGIC BOOLEANS
    private boolean m_isTurretHomed = false;
    public BooleanSupplier turretHomedSupp = () -> m_isTurretHomed;
    // private boolean m_isHoodHomed = false;

    /* SIM OBJECTS */
    private final FlywheelSim m_shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(2), kShooterMoI, kShooterGearing), DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), kTurretMoI, kTurretGearing), DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble("Shooter/Flywheel", "shooterVelocityRPS");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble("Shooter/Turret", "turretPositionRots");
    private final BooleanLogger log_onLeftSide = WaltLogger.logBoolean(kLogTab, "onLeftSide");

    // private final BooleanLogger log_exitBeamBreak =
    // WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    private final DoubleLogger log_shooterClosedLoopError = WaltLogger.logDouble("Shooter",
            "shooterClosedLoopError");

    private final DoubleLogger log_turretControlPos = WaltLogger.logDouble("Shooter/Turret", "turretControlPos");
    // private final DoubleLogger log_hoodControlPos = WaltLogger.logDouble("Shooter/Hood", "hoodControlPos");

    private final Tracer m_periodicTracer = new Tracer();

    StatusSignal<Double> sig_shooterCLErr = m_shooterA.getClosedLoopError();

    private final Command m_homingCommand = turretHomingCmd();

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_shooterA.getConfigurator().apply(kShooterATalonFXConfiguration);
        m_shooterB.getConfigurator().apply(kShooterBTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);
        // m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        m_shooterB.setControl(new Follower(kShooterA_CANID, MotorAlignmentValue.Opposed));

        m_turretEncoderA.getConfigurator();
        m_turretEncoderB.setPartyMode(5);

        CanandEventLoop.getInstance();

        sig_shooterCLErr.setUpdateFrequency(Hertz.of(50));

        Angle turretTurnPosition = m_turret.getPosition().getValue();
        m_flywheelVelocity = m_shooterA.getVelocity().getValue();

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;
        setDefaultCommand(m_homingCommand);

        m_turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(m_poseSupplier.get().rotateAround(
                        poseSupplier.get().getTranslation(), new Rotation2d(turretTurnPosition)))
                        .transformBy(kTurretTransform),
                fieldSpeedsSupplier);

        m_fuelSim = FuelSim.getInstance();

        trg_homingHallDirect.onTrue(Commands.sequence(
                Commands.runOnce(() -> {
                    m_homingCommand.cancel();
                    WaltLogger.timedPrint("FastHoming OK!!!");
                }),
                Commands.runOnce(() -> homingEventLoop.clear())));

        m_turret.setPosition(kInitPosition);

        initSim();
    }

    // public Command setShotCalcCmd(boolean enable) {
    //     return Commands.runOnce(() -> {
    //         m_useShotCalculator = enable;
    //     });
    // }

    // public void setShotCalc(boolean enable) {
    //     m_useShotCalculator = enable;
    // }

    public void setIntaking(boolean intaking) {
        m_holdTurretAtIntakePos = intaking;
    }

    public void setTurretLock(boolean locked) {
        m_turretLocked = locked;
        if (m_turretLocked) {
            m_turretLockAngle = m_turret.getPosition().getValue();
        }
    }

    public Command setTurretLockCmd(boolean locked) {
        return runOnce(() -> setTurretLock(locked));
    }

    // ---SHOOTER (Velocity Control)
    public Command setShooterVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setShooterVelocity(RPS));
    }

    public Command setShooterVelocityCmdSupp(Supplier<AngularVelocity> supp_RPS) {
        return runOnce(() -> setShooterVelocity(supp_RPS.get()));
    }

    public void setShooterVelocitySupp(Supplier<AngularVelocity> supp_RPS) {
        run(() -> setShooterVelocity(supp_RPS.get()));
    }

    public Command shootFromCalc() {
        return runOnce(() -> setShooterVelocity(m_calcFlywheelVelocity.get()));
    }

    public void setShooterVelocity(AngularVelocity RPS) {
        if (RPS.isEquivalent(RotationsPerSecond.zero())) {
            m_shooterA.setControl(m_neutralOutReq);
        } else {
            m_shooterA.setControl(m_velocityRequest.withVelocity(RPS));
        }
    }


    // for TestingDashboard
    public Command setShooterVelocityCmd(DoubleSubscriber sub_RPS) {
        return run(() -> setShooterVelocity(RotationsPerSecond.of(sub_RPS.get())));
    }

    public boolean isShooterSpunUp() {
        sig_shooterCLErr.refresh();
        log_shooterClosedLoopError.accept(sig_shooterCLErr.getValueAsDouble());

        boolean isNear = sig_shooterCLErr.isNear(0, 3);

        log_spunUp.accept(isNear);
        return isNear;
    }

    // ---TURRET (Motionmagic Angle Control)
    public Command setTurretPosCmd(Angle rots) {
        return runOnce(() -> setTurretPos(rots));
    }

    public void setTurretPos(Angle rots) {
        if (!m_isTurretHomed) { return; }
        m_turret.setControl(m_PVRequest.withPosition(rots));
        log_turretControlPos.accept(rots.in(Rotations));
    }

    public void setTurretNeutralMode(NeutralModeValue value) {
        m_turret.setNeutralMode(value);
    }

    // for TestingDashboard
    public Command setTurretPositionCmd(DoubleSubscriber sub_rots) {
        return run(() -> setTurretPos(Rotations.of(sub_rots.get())));
    }

    // // ---HOOD
    // public void setHoodPos(Angle degs) {
    //     m_hood.setControl(m_hoodPVRequest.withPosition(degs));
    //     log_hoodControlPos.accept(degs.in(Degrees));
    // }

    // public Command setHoodPosCmd(Angle degs) {
    //     return runOnce(() -> setHoodPos(degs));
    // }

    // //for TestingDashboard
    // public Command setHoodPositionCmd(DoubleSubscriber sub_degs) {
    //     return run(() -> setHoodPos(Degrees.of(sub_degs.get())));
    // }

    /* GETTERS */
    public AngularVelocity getShooterVelocity() {
        return m_shooterA.getVelocity().getValue();
    }

    // private Angle getHoodAngle() {
    //     return Degrees.zero();
    // }

    private Angle getTurretPosition() {
        return m_turret.getPosition().getValue();
    }

    public Supplier<AngularVelocity> getFlywheelCalcVelocity() {
        return m_calcFlywheelVelocity;
    }

    /* SIMULATION */
    public boolean simAbleToIntake() {
        return canIntake();
    }

    public void simIntake() {
        intakeFuel();
    }

    /**
     * @return true if robot can store more fuel
     */
    public boolean canIntake() {
        return m_fuelStored < kHopperCapacity;
    }

    public void intakeFuel() {
        m_fuelStored++;
    }

    // /**
    //  * Launches SIMULATION FUEL™ at the current Flywheel Velocity, current Hood
    //  * Angle, and the
    //  * current Turret Position.
    //  */
    // public void launchFuel() {
    //     if (m_fuelStored == 0)
    //         return;
    //     // m_fuelStored--;

    //     AngularVelocity flywheelVelocity = getShooterVelocity(); // current flywheel velocity
    //     Angle hoodAngle = Degrees.of(90).minus(getHoodAngle()); // current hood angle (need to subtract from 90 to get
    //                                                             // an accurate launching angle)
    //     Angle turretPosition = getTurretPosition(); // current turret position
    //     LinearVelocity flywheelLinearVelocity = ShotCalculator
    //             .angularToLinearVelocity(flywheelVelocity, kFlywheelRadius);

    //     m_fuelSim.launchFuel(flywheelLinearVelocity, hoodAngle, turretPosition,
    //             kTurretTransform.getMeasureZ()); // launch from where the turret *should* be
    // }

    // TODO: update orientation values (if needed)
    private void initSim() {
        WaltMotorSim.initSimFX(m_shooterA, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX60);
        WaltMotorSim.initSimFX(m_turret, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX44);
    }

    /* ShootOnTheMove™ */

    public record AzimuthCalcDetails(Pose3d desiredAimPose, Pose3d currentAimPose, double rawDesiredRotations, double safeDesiredRotations) {}

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
    public static Angle calculateAzimuthAngle(Translation3d target, Pose2d robotPose, Angle turretPosition,
            Consumer<AzimuthCalcDetails> logger) {
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
        // sub-360 cope calc
        // boolean calculated = false;
        // if (kTurretMaxRotsFromHome.times(2).magnitude() <
        // Rotations.of(1).magnitude()) {
        // snapbackSafeAngleRotations = MathUtil.clamp(angleRotations,
        // kTurretMinRots.in(Rotations), kTurretMaxRots.in(Rotations));
        // calculated = true;
        // }

        // this is the snapback function, to make sure that you will always be tracking
        // and you will not go over your physical limits.
        if (turretPositionRots > 0 && angleRotations + 1 <= kTurretMaxRotsD) {
            snapbackSafeAngleRotations += 1;
        } else if (turretPositionRots < 0 && angleRotations - 1 >= kTurretMinRotsD) {
            snapbackSafeAngleRotations -= 1;
        }

        logger.accept(
                new AzimuthCalcDetails(desiredAimPose, currentAimPose, angleRotations, snapbackSafeAngleRotations));
        return Rotations.of(snapbackSafeAngleRotations);
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

    /**
     * Calculates the ideal shot to put the FUEL™ into the HUB™
     * Accounts for moving speeds
     * 
     * @param robotPose current Robot position.
     */
    private void calcAndSetShot(Pose2d robotPose, boolean staticShot, Angle turretPosition) {
        // How fast the robot is currently going, (CURRENT ROBOT VELOCITY)
        ChassisSpeeds fieldSpeeds = staticShot ? WpiK.kZeroChassisSpeeds : m_fieldSpeedsSupplier.get();
        m_periodicTracer.addEpoch("calculateShot/getFieldSpeeds");

        // TODO: once we finish our lerp, switch to the
        // iterativeMovingShotFromInterpolationMap method
        // The Calculated shot itself, according to the current robotPose, robotSpeeds,
        // and the currentTarget
        ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromInterpolationMap(robotPose, fieldSpeeds,
                m_currentTarget, 3);
        m_periodicTracer.addEpoch("calculateShot/iterativeMovingShot");

        // The turret angle according to the Calculated shot
        log_calculatedShotTarget.accept(calculatedShot.getTarget());
        Angle azimuthAngle = Shooter.calculateAzimuthAngle(calculatedShot.getTarget(), robotPose, turretPosition,
                (AzimuthCalcDetails details) -> {
                    log_desiredAimPose.accept(details.desiredAimPose);
                    log_currentAimPose.accept(details.currentAimPose);
                    log_rawDesiredTurretRot.accept(details.rawDesiredRotations);
                    log_desiredTurretRot.accept(details.safeDesiredRotations);
                });
        m_periodicTracer.addEpoch("calculateShot/calculateAzimuthAngle");

        // Sets the TurretPosition to the Calculated TurretAngle
        if (m_turretLocked) {
            setTurretPos(m_turretLockAngle);
        } else {
            if (m_holdTurretAtIntakePos) {
                setTurretPos(Rotations.of(-0.250));
            } else {
                setTurretPos(azimuthAngle);
            }
        }
        // Sets the HoodPosition to the Calculated HoodAngle
        // m_hood.setHoodPos(Degrees.of(calculatedShot.getHoodAngle().in(Degrees)));

        m_calcTurret = azimuthAngle;
        m_calcFlywheelVelocity = () -> ShotCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(),
                kFlywheelRadius);
        m_periodicTracer.addEpoch("calculateShot/setOutputs");
    }

    /**
     * Version of calculateShot where, FOR TESTING, the turret will align to the
     * target.
     * 
     * @param robotPose current Robot position
     */
    // private void calculateTurretAngle(Pose2d robotPose) {
    // ChassisSpeeds fieldSpeeds = m_fieldSpeedsSupplier.get();

    // ShotData calculatedShot =
    // ShotCalculator.iterativeMovingShotFromFunnelClearance(
    // robotPose,
    // fieldSpeeds,
    // m_currentTarget,
    // 3);
    // Angle azimuthAngle = ShotCalculator.calculateAzimuthAngle(
    // robotPose,
    // calculatedShot.getTarget(),
    // m_turret.getPosition().getValue());
    // setTurretPos(azimuthAngle);

    // m_calcTurret = azimuthAngle;
    // }

    /**
     * Tells us whether or not our Turret, Hood, and Flywheel are at their desired
     * position with a certain amount of tolerance relative to the object
     * 
     * Hood Tolerance: .5 degrees. Turret Tolerance: __ rotations. Flywheel
     * Tolerance: __ RPS.
     * 
     * @return
     */
    // public boolean atPosition() {
    // var turretAtPos = getTurretPosition().isNear(m_calcTurret, Degrees.of(1));
    // //TODO: get the turret tolerance
    // var flywheelAtPos = getShooterVelocity().isNear(m_calcFlywheelVelocity,
    // RotationsPerSecond.of(1)); //TODO: get the flywheel tolerance
    // var hoodAtPos =
    // m_hoodEncoder.getVelocity().getValue().isNear(RotationsPerSecond.of(0),
    // 0.01); //is this right buh
    // return turretAtPos && flywheelAtPos && hoodAtPos;
    // }

    /* PERIODICS */
    @Override
    public void periodic() {
        m_periodicTracer.addEpoch("Entry (Unused Time)");

        // Cache all signals at the top so every consumer in this loop sees the same
        // values
        Angle turretTurnPosition = m_turret.getPosition().getValue();
        m_flywheelVelocity = m_shooterA.getVelocity().getValue();
        Pose2d pose = m_poseSupplier.get();
        m_periodicTracer.addEpoch("Getting Values");

        m_currentTarget = calculateTarget(pose);
        log_globalShotTarget.accept(m_currentTarget);

        if (m_isTurretHomed && m_hood.isHoodHomed()) {
            calcAndSetShot(pose, true, turretTurnPosition);
            m_periodicTracer.addEpoch("Calculating Shot");
        }

        log_shooterVelocityRPS.accept(m_flywheelVelocity.in(RotationsPerSecond));
        log_turretPositionRots.accept(turretTurnPosition.in(Rotations));
        log_spunUp.accept(isShooterSpunUp());
        log_onLeftSide.accept(m_onLeftSide);
        log_calcFlywheelVelocity.accept(m_calcFlywheelVelocity.get().in(RotationsPerSecond));
        log_calcTurretPos.accept(m_calcTurret.in(Rotations));

        m_periodicTracer.addEpoch("Logging");

        // m_periodicTracer.printEpochs();
    }

    public void fastPeriodic() {
        homingEventLoop.poll();
        log_turretHomed.accept(m_isTurretHomed);
        log_turretHomingHall.accept(trg_homingHallDirect);
    }

    @Override
    public void simulationPeriodic() {
        // m_turretVisualizer.updateFuel(
        //         ShotCalculator.angularToLinearVelocity(m_flywheelVelocity, kFlywheelRadius),
        //         getHoodAngle());

        WaltMotorSim.updateSimFX(m_shooterA, m_shooterSim);
        WaltMotorSim.updateSimFX(m_turret, m_turretSim);
        // WaltMotorSim.updateSimServo(m_hood, m_hoodSim);
    }

    public Command turretHomingCmd() {
        Runnable init = () -> {
            WaltLogger.timedPrint("TurretHoming BEGIN");
            m_turret.setControl(m_VoltageReq.withOutput(kHomingVoltage));
            m_isTurretHomed = false;
            log_turretHomed.accept(m_isTurretHomed);
        };

        Consumer<Boolean> end = (Boolean interrupted) -> {
            if (interrupted) {
                m_turret.setControl(m_BrakeReq);
                log_turretHomed.accept(m_isTurretHomed);
                WaltLogger.timedPrint("TurretHoming INTERRUPTED!!!");
                return;
            }

            m_turret.setPosition(kHomePosition); // Flowkirkentologicalexpialibrostatenuinely
            m_turret.setControl(m_BrakeReq);
            removeDefaultCommand();
            m_isTurretHomed = true;
            log_turretHomed.accept(m_isTurretHomed);
        };

        BooleanSupplier isFinished = () -> {
            return !m_turretHomingHall.get() || m_isTurretHomed;
        };

        return new FunctionalCommand(init, () ->{}, end, isFinished, this);
    }

    // public Command hoodCurrentSenseHomingCmd(){
    //     Runnable init = () -> {
    //         m_hood.setControl(m_hoodZeroReq.withOutput(kHomingVoltage));
    //         m_isHoodHomed = false;
    //         log_hoodHomed.accept(m_isHoodHomed);
    //     };

    //     Consumer<Boolean> end = (Boolean interrupted) -> {
    //         if (interrupted) {
    //             m_hood.setControl(m_BrakeReq);
    //             WaltLogger.timedPrint("HoodHoming INTERRUPTED!!!!");
    //             log_hoodHomed.accept(m_isHoodHomed);
    //             return;
    //         }

    //         m_hood.setPosition(kHoodHomePosition);
    //         m_hood.setControl(m_BrakeReq);
    //         removeDefaultCommand();
    //         m_isHoodHomed = true;
    //         log_hoodHomed.accept(m_isHoodHomed);
    //     };

    //     BooleanSupplier isFinished = () -> {
    //         return m_isHoodHomed;
    //     };

    //     return new FunctionalCommand(init, () -> {}, end, isFinished, this);
    // }

    public Command homingCmds() {
        return Commands.sequence(
            // hoodCurrentSenseHomingCmd().asProxy(),
            turretHomingCmd().asProxy()
        );
    }
}