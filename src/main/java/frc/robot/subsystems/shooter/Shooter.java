
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;

import org.wpilib.system.Timer;
import org.wpilib.system.Tracer;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.Coroutine;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterCalc.ShotCalcOutputs;
import frc.util.SignalManager;
import frc.util.WaltMotorSim;
import frc.util.WaltTunable;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends Mechanism {
    // NT-tunable overrides for LERP table building (default off)
    private static final WaltTunable kShooterRPSOverride =
        new WaltTunable("/Shooter/shooterRPSOverride", kShooterRPSd);
    private static final WaltTunable kHoodRotsOverride =
        new WaltTunable("/Shooter/hoodRotsOverride", 0.0);
    private static final double kHoodLockedPosRots = Rotations.of(0.721).magnitude();

    private final Tracer m_periodicTracer = new Tracer();
    /* VARIABLES */
    // boolean m_useShotCalculator = true;

    private double m_currentFlywheelVelocityRotPerSec;
    private double m_latestFlywheelAccelerationRotPerSec;
    private boolean m_isShooterSpunUp = false;
    private boolean m_shotDropSeen = false;

    private final Timer m_shotRecoveryTimer = new Timer();

    private int m_fuelStored = 8;

    // private final TurretVisualizer m_turretVisualizer;
    // private final FuelSim m_fuelSim;

    // ---MOTORS + CONTROL REQUESTS
    private final TalonFX m_shooterA = new TalonFX(kShooterA_CANID, Constants.kShooterBus); // X44
    private final TalonFX m_shooterB = new TalonFX(kShooterB_CANID, Constants.kShooterBus); // X44
    // private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VelocityTorqueCurrentFOC m_veloTQFOCReq = new VelocityTorqueCurrentFOC(0).withSlot(1);
    private final CoastOut m_motorIdleReq = new CoastOut();

    private final Supplier<SwerveDriveState> m_threadsafeSwerveSup;

    public final Hood m_hood;
    public final Turret m_turret;

    // thread copde
    private volatile double m_latestTurretPositionRots = 0.0;
    private final ShooterCalc m_shooterCalc;

    private double m_calcFlywheelVelocityRotPerSec = kShooterRPSd;
    private double m_calcHoodRots = kHoodRotsd;
    private double m_driverRPSTweak = 0.0;
    private double m_shotConfidence = 0.0; // updated every cycle from the shot calculator

    // threshold for isShotConfident() — not gating anything yet, just for telemetry
    private static final double kMinShotConfidence = 30.0;

    private int m_ballsShot = 0;

    private final StatusSignal<Double> sig_shooterCLErr = m_shooterA.getClosedLoopError();
    private final StatusSignal<AngularVelocity> sig_shooterAVelo = m_shooterA.getVelocity();
    private final StatusSignal<AngularAcceleration> sig_shooterAAccel = m_shooterA.getAcceleration();
    private final StatusSignal<ControlModeValue> sig_shooterACtrlMode = m_shooterA.getControlMode();

    // ---LOGIC BOOLEANS
    private final Trigger trg_inShootCtrlMode = new Trigger(() -> {return sig_shooterACtrlMode.getValue() == ControlModeValue.VelocityVoltage; });
    private final Trigger trg_ballDetected = new Trigger(() -> detectShot()).and(trg_inShootCtrlMode);
    private final Trigger trg_shotDropSeen = new Trigger(() -> m_shotDropSeen);
    private final Trigger trg_debounceHit = new Trigger(() -> m_shotRecoveryTimer.hasElapsed(1.2));

    private final Trigger trg_ballShotDebounced = trg_inShootCtrlMode
        .and(trg_shotDropSeen)
        .and(trg_ballDetected.negate())
        .and(trg_debounceHit);

    /* SIM OBJECTS */

    // 2027-TODO: figure out new LinearSystem generator!!!
    // private final FlywheelSim m_shooterSim = new FlywheelSim(LinearSystem.createFlywheelSystem(
            // DCMotor.getKrakenX44(2), kShooterMoI, kShooterGearing), DCMotor.getKrakenX60Foc(2) // returns gearbox
    // );

    // private final DCMotorSim m_turretSim = new DCMotorSim(LinearSystem.createDCMotorSystem(
    //         DCMotor.getKrakenX44Foc(1), kTurretMoI, kTurretGearing), DCMotor.getKrakenX44Foc(1) // returns gearbox
    // );

    /* LOGGERS */
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble("Shooter/Flywheel", "velocityRPS");
    private final DoubleLogger log_shooterAccelRPS = WaltLogger.logDouble("Shooter/Flywheel", "accelRPS");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble("Turret", "positionRots");
    private final DoubleLogger log_turretPositionRobotRelativeRots = WaltLogger.logDouble("Turret", "positionRobotRelativeRots");

    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");
    // private final BooleanLogger log_canTurretShoot = WaltLogger.logBoolean(kLogTab, "canTurretShoot");

    private final DoubleLogger log_shooterClosedLoopError = WaltLogger.logDouble("Shooter/Flywheel", "closedLoopError");

    private final BooleanLogger log_ballDetected = WaltLogger.logBoolean(kLogTab, "ballDetected");
    private final BooleanLogger log_ballShotDebounce = WaltLogger.logBoolean(kLogTab, "ballShot");

    private final DoubleLogger log_ballsShot = new DoubleLogger("Shooter/Flywheel", "balls shot");
    private final DoubleLogger log_calcFlywheelVelocity = new DoubleLogger("Shooter/Flywheel", "calcFlywheelVelocity");
    private final DoubleLogger log_driverAddedRPS = WaltLogger.logDouble(kLogTab, "driverAddedRPS");
    private final DoubleLogger log_shotConfidence = WaltLogger.logDouble(kLogTab, "shotConfidence");
    private final BooleanLogger log_shotConfident = WaltLogger.logBoolean(kLogTab, "shotConfident");

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<SwerveDriveState> threadsafeSwerveStateSup, Supplier<ChassisVelocities> fieldSpeedsSupplier) {
        m_hood = new Hood();
        m_turret = new Turret();
        m_threadsafeSwerveSup = threadsafeSwerveStateSup;
        m_shooterCalc = new ShooterCalc(m_threadsafeSwerveSup, () -> m_latestTurretPositionRots);

        m_shooterCalc.shouldUseStaticShot(kUseStaticShot);

        m_shooterA.getConfigurator().apply(kShooterATalonFXConfiguration);
        m_shooterB.getConfigurator().apply(kShooterBTalonFXConfiguration);
        // m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        m_shooterB.setControl(new Follower(kShooterA_CANID, MotorAlignmentValue.Opposed));

        sig_shooterCLErr.setUpdateFrequency(Hertz.of(50));

        SignalManager.register(Constants.kShooterBus, sig_shooterAVelo, sig_shooterCLErr, sig_shooterACtrlMode, sig_shooterAAccel);

        m_currentFlywheelVelocityRotPerSec = sig_shooterAVelo.getValueAsDouble();
        m_latestFlywheelAccelerationRotPerSec = sig_shooterAAccel.getValueAsDouble();

        // Trigger bindings (no mechanism requirement)
        trg_ballDetected.onTrue(
            Command.noRequirements(co -> {
                m_shotDropSeen = true;
                m_shotRecoveryTimer.restart();
                m_ballsShot++;
            }).named("OnBallDetected"));
        trg_ballDetected.onFalse(
            Command.noRequirements(co -> {
                m_shotRecoveryTimer.restart();
            }).named("OnBallLost"));
        trg_inShootCtrlMode.onFalse(
            Command.noRequirements(co -> {
                m_shotDropSeen = false;
                m_shotRecoveryTimer.stop();
                m_shotRecoveryTimer.reset();
            }).named("OnShootCtrlModeExit"));

        // m_turretVisualizer = new TurretVisualizer(
        //         () -> new Pose3d(m_poseSupplier.get().rotateAround(
        //                 poseSupplier.get().getTranslation(), new Rotation2d(turretTurnPosition)))
        //                 .transformBy(kTurretTransform),
        //         fieldSpeedsSupplier);

        // m_fuelSim = FuelSim.getInstance();
        initSim();

    // replaces @Override periodic()
        Scheduler.getDefault().addPeriodic(this::sideloadedPeriodic);
    }


    /** set flywheel to a fixed velocity, then release the mechanism. */
    public Command setShooterVelocityCmd(AngularVelocity RPS) {
        return run(co -> setShooterVelocity(RPS))
            .named("SetShooterVelocity");
    }

    /** set flywheel velocity from a supplier. */
    public Command setShooterVelocityCmdSupp(Supplier<AngularVelocity> supp_RPS) {
        return run(co -> setShooterVelocity(supp_RPS.get()))
            .named("SetShooterVelocitySupp");
    }

    /** drive the flywheel continuously from the shot calculator every cycle. */
    public Command shootFromCalc() {
        return runRepeatedly(() -> setShooterVelocity(m_calcFlywheelVelocityRotPerSec))
            .named("ShootFromCalc");
    }

    // TODO: migrate hood to Mechanism, then use m_hood.runRepeatedly() with proper requirement
    /** Continuous: drive the hood position from the shot calculator every cycle. */
    public Command hoodFromCalc() {
        return Command.noRequirements(co -> {
            while (co.yield()) {
                m_hood.setHoodPos(m_calcHoodRots);
            }
        }).named("HoodFromCalc");
    }

    // TODO: migrate hood to Mechanism, then use m_hood.runRepeatedly() with proper requirement
    /** Continuous: hold the hood at the halfway position. */
    public Command hoodToHalfway() {
        return Command.noRequirements(co -> {
            while (co.yield()) {
                m_hood.setHoodPos(kHoodRotsHalfwayD);
            }
        }).named("HoodToHalfway");
    }

    public Command driverRPSIncreaseWhileHeldCmd() {
        return run(co -> {
            m_driverRPSTweak += kDriverRPSIncreaseD;
            log_driverAddedRPS.accept(m_driverRPSTweak);
            co.park();
        }).whenCanceled(() -> m_driverRPSTweak = 0)
          .named("DriverRPSIncreaseWhileHeld");
    }

    /** bump the driver RPS tweak by +/-5% of calc velocity */
    public Command driverRPSAlterDynamic(boolean increase) {
        return Command.noRequirements(co -> {
            m_driverRPSTweak = increase ? (m_calcFlywheelVelocityRotPerSec * 0.05) : (m_calcFlywheelVelocityRotPerSec * -0.05);
            log_driverAddedRPS.accept(m_driverRPSTweak);
        }).named("DriverRPSAlterDynamic");
    }

    /** bump the driver RPS tweak by a fixed step. */
    public Command driverRPSAlterStatic(boolean increase) {
        return Command.noRequirements(co -> {
            m_driverRPSTweak += kDriverRPSIncreaseD * (increase ? 1 : -1);
            log_driverAddedRPS.accept(m_driverRPSTweak);
        }).named("DriverRPSAlterStatic");
    }

    /** zero out the driver RPS tweak. */
    public Command driverResetRPSAlter() {
        return Command.noRequirements(co -> m_driverRPSTweak = 0)
            .named("DriverResetRPSAlter");
    }

    // ==================== HARDWARE SETTERS ====================

    public void setShooterVelocity(AngularVelocity RPS) {
        setShooterVelocity(RPS.in(RotationsPerSecond));
    }

    public void setShooterVelocity(double rotPerSec) {
        if (rotPerSec == 0) {
            // cope to get clErr to 0
            m_shooterA.setControl(m_veloTQFOCReq.withVelocity(0));
            m_shooterA.setControl(m_motorIdleReq);
        } else {
            m_shooterA.setControl(m_veloTQFOCReq.withVelocity(rotPerSec));
        }
    }

    private void refreshShooterSpunUp() {
        log_shooterClosedLoopError.accept(sig_shooterCLErr.getValueAsDouble());
        m_isShooterSpunUp = sig_shooterCLErr.isNear(0, 0.5);
    }

    public boolean isShooterSpunUp() {
        return m_isShooterSpunUp;
    }

    // does NOT block firing — just tells you if the shot calc thinks this is a good shot
    public boolean isShotConfident() {
        return m_shotConfidence >= kMinShotConfidence;
    }

    public double getShotConfidence() {
        return m_shotConfidence;
    }

    /* GETTERS */
    public double getShooterVelocityRotPerSec() {
        return m_currentFlywheelVelocityRotPerSec;
    }

    public double getShooterDesiredRotPerSec() {
        return m_calcFlywheelVelocityRotPerSec;
    }

    public DoubleSupplier getShooterDesiredRotPerSecSupp() {
        return () -> m_calcFlywheelVelocityRotPerSec;
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

    private boolean detectShot() {
        boolean accelDrop = m_latestFlywheelAccelerationRotPerSec <= -3.0;
        return accelDrop;
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
    // }

    public Trigger getBallShotDebounceTrg() {
        return trg_ballShotDebounced;
    }

    // TODO: update orientation values (if needed)
    private void initSim() {
        WaltMotorSim.initSimFX(m_shooterA, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX60);
    }

    // ==================== SIDELOADED PERIODIC ====================
    // runs every scheduler cycle via Scheduler.addPeriodic() - NOT tied to mechanism ownership
    private void sideloadedPeriodic() {
        m_periodicTracer.addEpoch("Entry (Unused Time)");

        // Cache all signals at the top so every consumer in this loop sees the same values
        // THIS IS USED SNEAKILY BY SHOTCALC DO NOT MOVE THIS
        m_latestTurretPositionRots = m_turret.getCurrTurretPos();
        m_currentFlywheelVelocityRotPerSec = sig_shooterAVelo.getValueAsDouble();
        m_latestFlywheelAccelerationRotPerSec = sig_shooterAAccel.getValueAsDouble();

        ShotCalcOutputs calcData = m_shooterCalc.getLatestShotCalcOutputs();
        m_shotConfidence = calcData.shotConfidence();

        m_periodicTracer.addEpoch("Stashing ShotCalc data");
        log_shooterClosedLoopError.accept(sig_shooterCLErr.getValueAsDouble());

        // set turret reference
        if (m_turret.isTurretHomed()) {
            var turretReference = calcData.turretReferenceRots();
            // set outputs
            var turretVelocityFF = calcData.turretCalcDetails().turretVelocityFF();
            if (m_turret.getTurretLocked()) {
                // m_turret.setTurretPos(m_turret.getTurretLockAngleRots(), 0.0);
                m_calcFlywheelVelocityRotPerSec = kShooterRPSd;
            } else {
                // NOT LOCKED
                if (m_turret.getHoldTurretAtIntake()) {
                    // m_turret.setTurretPos(Rotations.of(-0.250));
                } else {
                    // m_turret.setTurretPos(turretReference, turretVelocityFF);
                    m_calcFlywheelVelocityRotPerSec = kShooterRPSOverride.enabled()
                        ? kShooterRPSOverride.get()
                        : calcData.shooterReferenceRps();
                    if (kAllowDriverRPSTweak) { // ENABLE THIS TO ALLOW DRIVER RPS TWEAK
                        m_calcFlywheelVelocityRotPerSec += m_driverRPSTweak;
                        m_calcFlywheelVelocityRotPerSec = Math.clamp(m_calcFlywheelVelocityRotPerSec, 0, kShooterMaxRPSd);    //clamp here or clamp only when setShooterVel is called?
                    }
                }
            }
        }

        if (m_hood.isHoodHomed()) {
            double hoodReference = calcData.hoodReferenceRots();
            if (m_turret.getTurretLocked()) {
                m_calcHoodRots = kHoodLockedPosRots;
                // m_hood.setHoodPos(kHoodLockedPosRots);
            } else {
                if (!m_turret.getHoldTurretAtIntake()) {
                m_calcHoodRots = kHoodRotsOverride.enabled()
                    ? kHoodRotsOverride.get()
                    : hoodReference;
                    // m_hood.setHoodPos(kHoodRotsOverride.enabled()
                    // ? kHoodRotsOverride.get()
                    // : hoodReference);
                }
            }
        }

        m_periodicTracer.addEpoch("Setting Hood & Turret References");

        refreshShooterSpunUp();

        m_periodicTracer.addEpoch("Refresh shooterSpunUp");

        log_turretPositionRobotRelativeRots.accept(kDriverRPSIncreaseD);
        log_ballsShot.accept(m_ballsShot);
        log_shooterVelocityRPS.accept(m_currentFlywheelVelocityRotPerSec);
        log_shooterAccelRPS.accept(m_latestFlywheelAccelerationRotPerSec);
        log_turretPositionRots.accept(m_latestTurretPositionRots);
        log_spunUp.accept(m_isShooterSpunUp);
        log_calcFlywheelVelocity.accept(m_calcFlywheelVelocityRotPerSec);
        log_ballDetected.accept(trg_ballDetected.getAsBoolean());
        log_ballShotDebounce.accept(trg_ballShotDebounced.getAsBoolean());
        log_shotConfidence.accept(m_shotConfidence);
        log_shotConfident.accept(isShotConfident());

        // m_periodicTracer.addEpoch("Logging");

        // m_periodicTracer.printEpochs();
    }

    // 2027-TODO: figure out new LinearSystem generator!!!
    // private void simulationPeriodic() {
    //     WaltMotorSim.updateSimFX(m_shooterA, m_shooterSim);
    // }
}
