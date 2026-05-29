package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.kRioBus;
import static frc.robot.Constants.IntakeK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltMotorSim;
import frc.robot.Robot;
import frc.util.SignalManager;
import frc.util.WaltLogger;

/*
 * Intake
 *
 * The intake is a ground-level collector that picks up fuel balls during a match.
 * It has two parts that work together:
 *
 *   Arm    — A pivoting arm that swings down so the rollers can reach balls on
 *            the ground. It's controlled by a single motor using MotionMagic
 *            (CTRE's trapezoidal motion profiling) so it moves smoothly to each
 *            target position without slamming.
 *
 *   Rollers — Two motors that spin to pull the ball in and pass it to the indexer.
 *             Motor B mirrors motor A (via Follower) but runs in the opposite
 *             direction so both rollers push the ball the same way.
 *
 * At the start of every match/auto, the arm needs to home itself — it doesn't
 * have an absolute encoder, so it slowly drives into a mechanical hard stop,
 * detects the stall via current + velocity sensing, and zeros the encoder there.
 * See intakeArmCurrentSenseHoming().
 */
public class Intake extends SubsystemBase {

    // ---- motors ----

    private final TalonFX m_intakeArm = new TalonFX(kIntakeArmCANID);      // arm pivot (KrakenX44Foc on Rio bus)
    private final TalonFX m_intakeRollersA = new TalonFX(kIntakeRollersA_CANID); // primary roller (KrakenX60Foc)
    private final TalonFX m_intakeRollersB = new TalonFX(kIntakeRollersB_CANID); // follower roller (KrakenX60Foc)

    // ---- control requests ----
    // Pre-created and reused every loop to avoid unnecessary object allocation.

    // MotionMagicVoltage: moves the arm smoothly to a target position.
    // MotionMagic generates a trapezoidal velocity profile internally (accelerate, cruise, decelerate)
    // so the arm doesn't slam into positions and cause mechanical stress or brownouts.
    private MotionMagicVoltage m_MMVReq = new MotionMagicVoltage(0).withEnableFOC(true);

    // VelocityVoltage: holds the rollers at a specific speed (RPS) using closed-loop control.
    private VelocityVoltage m_VelVoltReq = new VelocityVoltage(0).withEnableFOC(true);

    // VoltageOut: applies raw voltage directly to the arm motor. Used only during homing,
    // where we intentionally drive into the hard stop at a low, safe voltage.
    private VoltageOut m_voltsReq = new VoltageOut(0).withEnableFOC(true);

    // A second VoltageOut specifically for zeroing — kept separate from m_voltsReq so
    // periodic logging (which reads m_voltsReq.Output) isn't contaminated by homing values.
    private VoltageOut m_intakeArmZeroingReq = new VoltageOut(0);

    // ---- status signals ----

    private final StatusSignal<Current> sig_intakeArmStatorCurrent = m_intakeArm.getStatorCurrent();
    private final StatusSignal<AngularVelocity> sig_intakeArmVelo = m_intakeArm.getVelocity();
    private final StatusSignal<AngularVelocity> sig_intakeRollersAVelo = m_intakeRollersA.getVelocity();
    private final StatusSignal<Angle> sig_intakeArmPos = m_intakeArm.getPosition();
    private final StatusSignal<Boolean> sig_intakeArmMMAtTarget = m_intakeArm.getMotionMagicAtTarget();

    // ---- homing detection helpers ----
    // These two conditions together confirm the arm has hit the mechanical hard stop:
    //   1. Stator current spikes (motor is stalled / fighting the hard stop)
    //   2. Arm velocity is near zero (it's not moving anymore)

    private BooleanSupplier m_currentSpike = () -> sig_intakeArmStatorCurrent.getValueAsDouble() > 5.0;
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(sig_intakeArmVelo.getValueAsDouble()) < 0.005;
    // Looser velocity threshold used during the shimmy motion, which has intentional slow movement.
    private BooleanSupplier m_shimmyVeloIsNearZero = () -> Math.abs(sig_intakeArmVelo.getValueAsDouble()) < 0.05;

    // Debouncers filter out momentary spikes that don't represent a true stall.
    // kRising means the output only goes true after the input has been true for the set duration.
    private Debouncer m_currentDebouncer  = new Debouncer(0.100, DebounceType.kRising); // 100 ms
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising); // 125 ms

    private boolean m_isIntakeArmHomed = false;

    // Public supplier so other classes (e.g. WaltAdaptableAutonFactory) can wait on homing.
    public final BooleanSupplier intakeHomedSupp = () -> m_isIntakeArmHomed;

    // ---- simulation models ----
    // WPILib DCMotorSim models approximate the motor's physics during simulation.
    // These use the physical MOI and gear ratios from Constants so sim behavior
    // roughly matches the real robot.

    private final DCMotorSim m_intakeArmSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), kIntakeArmMOI, kIntakeArmGearing),
        DCMotor.getKrakenX60Foc(1)
    );

    private final DCMotorSim m_intakeRollersSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), kIntakeRollersMOI, kIntakeRollersGearing),
        DCMotor.getKrakenX60Foc(2)
    );

    // ---- loggers ----

    private final DoubleLogger log_intakeArmRots = WaltLogger.logDouble(kLogTab, "intakeArmRots");
    private final DoubleLogger log_targetIntakeArmRots = WaltLogger.logDouble(kLogTab, "targetIntakeArmRots");
    private final DoubleLogger log_intakeRollersRPS = WaltLogger.logDouble(kLogTab, "intakeRollersRPS");
    private final DoubleLogger log_targetIntakeRollersRPS = WaltLogger.logDouble(kLogTab, "targetIntakeRollersRPS");
    private final BooleanLogger log_isIntakeArmHomed = WaltLogger.logBoolean(kLogTab, "isIntakeArmHomed");


    // =============================================================
    // CONSTRUCTOR
    // =============================================================

    public Intake() {
        m_intakeArm.getConfigurator().apply(kIntakeArmConfiguration);
        m_intakeRollersA.getConfigurator().apply(kIntakeRollersAConfiguration);
        m_intakeRollersB.getConfigurator().apply(kIntakeRollersBConfiguration);

        // Motor B mirrors motor A but spins in the opposite direction so both rollers
        // push the ball the same way (they're physically mirrored on the robot).
        m_intakeRollersB.setControl(new Follower(kIntakeRollersA_CANID, MotorAlignmentValue.Opposed));

        SignalManager.register(kRioBus,
            sig_intakeArmStatorCurrent, sig_intakeArmVelo,
            sig_intakeRollersAVelo, sig_intakeArmPos, sig_intakeArmMMAtTarget);

        // On the real robot, start homing immediately on startup.
        // In sim, homing via current sensing doesn't work so we skip it.
        if (Robot.isReal()) {
            setDefaultCommand(intakeArmCurrentSenseHoming());
        }

        initSim();
    }

    private void initSim() {
        WaltMotorSim.initSimFX(m_intakeArm, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX44);
        WaltMotorSim.initSimFX(m_intakeRollersA, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX60);
    }


    // =============================================================
    // ARM POSITION CONTROL
    // =============================================================

    // These set the arm to a named position from the IntakeArmPosition enum.
    // Uses MotionMagic for smooth, profiled movement.
    public void setIntakeArmPos(IntakeArmPosition rots) {
        setIntakeArmPos(rots.rots);
    }

    public Command setIntakeArmPosCmd(IntakeArmPosition rots) {
        return setIntakeArmPosCmd(rots.rots);
    }

    public Command setIntakeArmPosCmd(Angle rots) {
        return runOnce(() -> setIntakeArmPos(rots));
    }

    public void setIntakeArmPos(Angle rots) {
        m_intakeArm.setControl(m_MMVReq.withPosition(rots));
    }

    // True when the arm has stopped moving (velocity near zero), used as a
    // "are we there yet" check during shimmy movements.
    public boolean isIntakeArmAtDest() {
        return m_shimmyVeloIsNearZero.getAsBoolean();
    }

    public void setIntakeArmNeutralMode(NeutralModeValue value) {
        m_intakeArm.setNeutralMode(value);
    }


    // =============================================================
    // ROLLER CONTROL
    // =============================================================

    // NOTE: despite the parameter name being "volts", this method actually does
    // velocity control. The volts value is treated as a 0-12V percentage and
    // mapped to a target RPS (0 V = 0 RPS, 12 V = max RPS). This gives a
    // convenient voltage-like API while still benefiting from closed-loop control.
    public void setIntakeRollersVelocity(double volts) {
        m_intakeRollersA.setControl(m_VelVoltReq.withVelocity(volts / 12 * kIntakeRollersMaxRPS.in(RotationsPerSecond)));
    }

    public Command setIntakeRollersVelocityCmd(double volts) {
        return runOnce(() -> setIntakeRollersVelocity(volts));
    }

    public Command startIntakeRollers(double volts) {
        return setIntakeRollersVelocityCmd(volts);
    }

    public Command stopIntakeRollers() {
        return setIntakeRollersVelocityCmd(0);
    }


    // =============================================================
    // HOMING
    // =============================================================

    /*
     * intakeArmCurrentSenseHoming
     *
     * Finds the arm's mechanical zero position without an absolute encoder.
     *
     * How it works:
     *   1. Drive the arm slowly toward the hard stop with a small negative voltage (-3.25 V).
     *   2. When the arm hits the stop, the motor stalls: current spikes AND velocity drops to zero.
     *   3. Both conditions are debounced (held for 100/125 ms) to filter out false positives
     *      from momentary bumps or noise during movement.
     *   4. Once both conditions are confirmed, zero the encoder at that position, mark homed,
     *      retract the arm, and remove this as the default command.
     *
     * The whole thing has a 3 s timeout so a broken sensor or stuck arm doesn't stall the robot.
     *
     * FunctionalCommand is WPILib's way of building a command inline from four lambda functions:
     *   init      — runs once when the command starts
     *   execute   — runs every loop while the command is active (nothing to do here)
     *   onEnd     — runs once when the command ends (whether it finished or was interrupted)
     *   isFinished — returns true when the command should stop
     */
    public Command intakeArmCurrentSenseHoming() {
        Runnable init = () -> {
            m_intakeArm.setControl(m_intakeArmZeroingReq.withOutput(-3.25)); // drive slowly into the hard stop
            m_isIntakeArmHomed = false;
            log_isIntakeArmHomed.accept(m_isIntakeArmHomed);
        };

        Runnable execute = () -> {};

        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_intakeArm.setControl(m_intakeArmZeroingReq.withOutput(0)); // stop applying voltage
            m_intakeArm.setPosition(0);                                   // zero the encoder here
            removeDefaultCommand();
            setIntakeArmPosCmd(IntakeArmPosition.RETRACTED);              // move to safe position
            m_isIntakeArmHomed = true;
            log_isIntakeArmHomed.accept(m_isIntakeArmHomed);
        };

        // Both current spike AND near-zero velocity must be true for their respective debounce
        // durations before we declare the arm is at the hard stop.
        BooleanSupplier isFinished = () ->
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) &&
            m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());

        return new FunctionalCommand(init, execute, onEnd, isFinished, this)
            .withTimeout(3)
            .withName("intakeArm homing");
    }


    // =============================================================
    // PERIODIC
    // =============================================================

    @Override
    public void periodic() {
        log_targetIntakeArmRots.accept(m_MMVReq.Position);
        log_targetIntakeRollersRPS.accept(m_voltsReq.Output);
        log_intakeRollersRPS.accept(sig_intakeRollersAVelo.getValueAsDouble());
        log_intakeArmRots.accept(sig_intakeArmPos.getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        WaltMotorSim.updateSimFX(m_intakeArm,      m_intakeArmSim);
        WaltMotorSim.updateSimFX(m_intakeRollersA, m_intakeRollersSim);
    }


    // =============================================================
    // ENUMS
    // =============================================================

    /*
     * IntakeArmPosition
     *
     * Named positions for the intake arm. All values are stored as both
     * degrees (for human readability) and rotations (for motor commands).
     * Note: these are mechanism rotations (arm angle), not motor rotations —
     * the 125:1 gear ratio is baked into the TalonFX feedback config so the
     * controller reports arm angle directly.
     *
     *   RETRACTED — arm is fully up, tucked inside the robot frame. Safe for driving.
     *   DEPLOYED  — arm is fully down to collect balls off the ground.
     *   SHIMMY    — partway down, used to agitate balls already inside the robot.
     *   SAFE      — just above DEPLOYED, used when approaching a ball cautiously.
     */
    public enum IntakeArmPosition {
        RETRACTED(Rotations.of(0.061514).in(Degrees)),
        DEPLOYED(Rotations.of(0.289062 * 0.86).in(Degrees)),
        SHIMMY(Rotations.of(0.126025).in(Degrees)),
        SAFE((DEPLOYED.rots.minus(Rotations.of(0.06))).in(Degrees));

        public Angle degs;
        public Angle rots;

        private IntakeArmPosition(double degs) {
            this.degs = Degrees.of(degs);
            this.rots = Rotations.of(this.degs.in(Rotations));
        }
    }
}
