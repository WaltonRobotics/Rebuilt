
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
// import com.reduxrobotics.canand.CanandEventLoop;
// import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterCalc.ShotCalcOutputs;
import frc.util.SignalManager;
import frc.util.WaltMotorSim;
import frc.util.WaltTunable;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    // NT-tunable overrides for LERP table building (default off)
    private static final WaltTunable kShooterRPSOverride =
        new WaltTunable("/Shooter/shooterRPSOverride", kShooterRPSd);
    private static final WaltTunable kHoodRotsOverride =
        new WaltTunable("/Shooter/hoodRotsOverride", 0.0);
    /* VARIABLES */
    // boolean m_useShotCalculator = true;

    private double m_latestFlywheelVelocityRotPerSec;
    private double m_latestFlywheelAccelerationRotPerSec;
    private boolean m_isShooterSpunUp = false;
    private boolean m_shotDropSeen = false;
    private boolean m_canTurretShoot = false;

    private final Timer m_shotRecoveryTimer = new Timer();

    private int m_fuelStored = 8;

    // private final TurretVisualizer m_turretVisualizer;
    // private final FuelSim m_fuelSim;

    // ---MOTORS + CONTROL REQUESTS
    private final TalonFX m_shooterA = new TalonFX(kShooterA_CANID, Constants.kShooterBus); // X44
    private final TalonFX m_shooterB = new TalonFX(kShooterB_CANID, Constants.kShooterBus); // X44
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final CoastOut m_motorIdleReq = new CoastOut();

    private final Supplier<SwerveDriveState> m_threadsafeSwerveSup;

    public final Hood m_hood;
    public final Turret m_turret;

    // thread copde
    private volatile double m_latestTurretPositionRots = 0.0;
    private final ShooterCalc m_shooterCalc;

    private double m_calcTurretRots = 0.0;
    private double m_calcFlywheelVelocityRotPerSec = kShooterRPSd;
    private double m_driverRPSTweak = 0.0;

    private int m_ballsShot = 0;

    private final StatusSignal<Double> sig_shooterCLErr = m_shooterA.getClosedLoopError();
    private final StatusSignal<AngularVelocity> sig_shooterAVelo = m_shooterA.getVelocity();
    private final StatusSignal<AngularAcceleration> sig_shooterAAccel = m_shooterA.getAcceleration();
    private final StatusSignal<ControlModeValue> sig_shooterACtrlMode = m_shooterA.getControlMode();

    // ---LOGIC BOOLEANS
    private final Trigger trg_inShootCtrlMode = new Trigger(() -> {return sig_shooterACtrlMode.getValue() == ControlModeValue.VelocityVoltage; });
    private final Trigger trg_ballDetected = new Trigger(() -> detectShot()).and(trg_inShootCtrlMode);
    private final Trigger trg_ballShotDebounced = trg_inShootCtrlMode
        .and(new Trigger(() -> m_shotDropSeen))
        .and(trg_ballDetected.negate())
        .and(new Trigger(() -> m_shotRecoveryTimer.hasElapsed(kBallDetectedDebounceTime)));

    /* SIM OBJECTS */
    private final FlywheelSim m_shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX44(2), kShooterMoI, kShooterGearing), DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), kTurretMoI, kTurretGearing), DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble("Shooter/Flywheel", "shooterVelocityRPS");
    private final DoubleLogger log_shooterAccelRPS = WaltLogger.logDouble("Shooter/Flywheel", "shooterAccelRPS");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble("Shooter/Turret", "turretPositionRots");
    private final DoubleLogger log_turretPositionRobotRelativeRots = WaltLogger.logDouble("Shooter/Turret", "turretPositionRobotRelativeRots");

    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");
    private final BooleanLogger log_canTurretShoot = WaltLogger.logBoolean(kLogTab, "canTurretShoot");

    private final DoubleLogger log_shooterClosedLoopError = WaltLogger.logDouble("Shooter", "shooterClosedLoopError");

    private final BooleanLogger log_ballDetected = WaltLogger.logBoolean(kLogTab, "ballDetected");
    private final BooleanLogger log_ballShot = WaltLogger.logBoolean(kLogTab, "ballShot");

    private final DoubleLogger log_ballsShot = new DoubleLogger("Shooter/Flywheel", "balls shot");
    private final DoubleLogger log_calcFlywheelVelocity = new DoubleLogger("Shooter/Flywheel", "calcFlywheelVelocity");
    private final DoubleLogger log_calcTurretPos = new DoubleLogger("Shooter/Turret", "calcTurretPos");
    private final DoubleLogger log_driverAddedRPS = WaltLogger.logDouble(kLogTab, "driverAddedRPS");

    /* CONSTRUCTOR */
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<SwerveDriveState> threadsafeSwerveStateSup, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
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

        m_latestFlywheelVelocityRotPerSec = sig_shooterAVelo.getValueAsDouble();
        m_latestFlywheelAccelerationRotPerSec = sig_shooterAAccel.getValueAsDouble();

        trg_ballDetected.onTrue(Commands.runOnce(() -> { m_shotDropSeen = true; m_shotRecoveryTimer.restart(); m_ballsShot++;}));
        trg_ballDetected.onFalse(Commands.runOnce(() -> { m_shotRecoveryTimer.restart(); }));
        trg_inShootCtrlMode.onFalse(Commands.runOnce(() -> { m_shotDropSeen = false; m_shotRecoveryTimer.stop(); m_shotRecoveryTimer.reset(); }));

        // m_turretVisualizer = new TurretVisualizer(
        //         () -> new Pose3d(m_poseSupplier.get().rotateAround(
        //                 poseSupplier.get().getTranslation(), new Rotation2d(turretTurnPosition)))
        //                 .transformBy(kTurretTransform),
        //         fieldSpeedsSupplier);

        // m_fuelSim = FuelSim.getInstance();
        initSim();
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
        return run(() -> setShooterVelocity(m_calcFlywheelVelocityRotPerSec));
    }

    public Command driverRPSIncreaseWhileHeldCmd() {
        return Commands.runOnce(() -> {
            driverRPSAlterStatic(true);
        }).finallyDo(() -> driverResetRPSAlter());
    }

    public Command driverRPSAlterDynamic(boolean increase) {
        return Commands.runOnce(() -> {
            m_driverRPSTweak = increase ? (m_calcFlywheelVelocityRotPerSec * 0.05) : (m_calcFlywheelVelocityRotPerSec * -0.05);
            log_driverAddedRPS.accept(m_driverRPSTweak);
        });
    }

    public Command driverRPSAlterStatic(boolean increase) {
        return Commands.runOnce(() -> {
            m_driverRPSTweak += kDriverRPSIncreaseD * (increase ? 1 : -1);
            log_driverAddedRPS.accept(m_driverRPSTweak);
        });
    }

    public Command driverResetRPSAlter() {
        return Commands.runOnce(() -> m_driverRPSTweak = 0);
    }

    public void setShooterVelocity(AngularVelocity RPS) {
        setShooterVelocity(RPS.in(RotationsPerSecond));
    }

    public void setShooterVelocity(double rotPerSec) {
        if (rotPerSec == 0) {
            // cope to get clErr to 0
            m_shooterA.setControl(m_velocityRequest.withVelocity(0));
            m_shooterA.setControl(m_motorIdleReq);
        } else {
            m_shooterA.setControl(m_velocityRequest.withVelocity(rotPerSec));
        }
    }

    private void refreshShooterSpunUp() {
        log_shooterClosedLoopError.accept(sig_shooterCLErr.getValueAsDouble());
        m_isShooterSpunUp = sig_shooterCLErr.isNear(0, 3);
    }

    /**
     * first checks if we're passing. If we're not passing, then we can shoot whenever
     * If we are passing, it checks if we're NOT in the unable-to-pass range. If we're not in it, then the turret can shoot!
     * 
     * lowk should be checking if we are IN THE RANGE rather than greater than the outsides, but this is cope for now cuz sadness
     */
    private void refreshCanTurretShoot() {
        if (ShooterCalc.isPassing().getAsBoolean()) {
            if (m_latestTurretPositionRots > kTurretMinNotAbleToPassRange && m_latestTurretPositionRots < kTurretMaxNotAbleToPassRange) {m_canTurretShoot = true;} else {m_canTurretShoot = false;}
            log_canTurretShoot.accept(m_canTurretShoot);
        } else {
            m_canTurretShoot = true;    //the turret can shoot anytime we are not passing
            log_canTurretShoot.accept(m_canTurretShoot);
        }
    }

    public boolean canTurretShoot() {
        return m_canTurretShoot;
    }

    public boolean isShooterSpunUp() {
        return m_isShooterSpunUp;
    }

    /* GETTERS */
    public double getShooterVelocityRotPerSec() {
        return m_latestFlywheelVelocityRotPerSec;
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
        boolean accelDrop = m_latestFlywheelAccelerationRotPerSec <= -20.0;
        return accelDrop;
    }

    public Trigger getBallShotTrg() {
        return trg_ballShotDebounced;
    }

    // TODO: update orientation values (if needed)
    private void initSim() {
        WaltMotorSim.initSimFX(m_shooterA, ChassisReference.CounterClockwise_Positive,
                TalonFXSimState.MotorType.KrakenX60);
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        m_turret.periodic();
        // m_periodicTracer.addEpoch("Entry (Unused Time)");

        // Cache all signals at the top so every consumer in this loop sees the same values
        // THIS IS USED SNEAKILY BY SHOTCALC DO NOT MOVE THIS
        m_latestTurretPositionRots = m_turret.getCurrTurretPos();
        m_latestFlywheelVelocityRotPerSec = sig_shooterAVelo.getValueAsDouble();
        m_latestFlywheelAccelerationRotPerSec = sig_shooterAAccel.getValueAsDouble();

        ShotCalcOutputs calcData = m_shooterCalc.getLatestShotCalcOutputs();

        sig_shooterCLErr.refresh();
        log_shooterClosedLoopError.accept(sig_shooterCLErr.getValueAsDouble());

        // set turret reference
        if (m_turret.isTurretHomed()) {
            var turretReference = calcData.turretReferenceRots();

            // set outputs
            var turretVelocityFF = calcData.turretCalcDetails().turretVelocityFF();
            if (m_turret.getTurretLocked()) {
                m_turret.setTurretPos(m_turret.getTurretLockAngleRots(), 0.0);
                m_calcFlywheelVelocityRotPerSec = kShooterRPSd;
            } else {
                if (m_turret.getHoldTurretAtIntake()) {
                    // m_turret.setTurretPos(Rotations.of(-0.250));
                } else {
                    m_turret.setTurretPos(turretReference, turretVelocityFF);
                    m_calcFlywheelVelocityRotPerSec = kShooterRPSOverride.enabled()
                        ? kShooterRPSOverride.get()
                        : calcData.shooterReferenceRps();
                    if (false) { // ENABLE THIS TO ALLOW DRIVER RPS TWEAK
                        m_calcFlywheelVelocityRotPerSec += m_driverRPSTweak;
                        m_calcFlywheelVelocityRotPerSec = MathUtil.clamp(m_calcFlywheelVelocityRotPerSec, 0, kShooterMaxRPSd);    //clamp here or clamp only when setShooterVel is called?
                    }
                }
            }
        }

        if (m_hood.isHoodHomed()) {
            double hoodReference = calcData.hoodReferenceRots();

            if (m_turret.getTurretLocked()) {
                m_hood.setHoodPos(kHoodLockRots_double);
            } else {
                if (!m_turret.getHoldTurretAtIntake()) {
                    m_hood.setHoodPos(kHoodRotsOverride.enabled()
                        ? kHoodRotsOverride.get()
                        : hoodReference);
                }
            }
        }

        refreshShooterSpunUp();
        refreshCanTurretShoot();

        log_turretPositionRobotRelativeRots.accept(kDriverRPSIncreaseD);
        log_ballsShot.accept(m_ballsShot);
        log_shooterVelocityRPS.accept(m_latestFlywheelVelocityRotPerSec);
        log_shooterAccelRPS.accept(m_latestFlywheelAccelerationRotPerSec);
        log_turretPositionRots.accept(m_latestTurretPositionRots);
        log_spunUp.accept(m_isShooterSpunUp);
        log_calcFlywheelVelocity.accept(m_calcFlywheelVelocityRotPerSec);
        log_calcTurretPos.accept(m_calcTurretRots);
        log_ballDetected.accept(trg_ballDetected.getAsBoolean());
        log_ballShot.accept(trg_ballShotDebounced.getAsBoolean());


        // m_periodicTracer.addEpoch("Logging");

        // m_periodicTracer.printEpochs();
    }

    @Override
    public void simulationPeriodic() {
        WaltMotorSim.updateSimFX(m_shooterA, m_shooterSim);
    }
}