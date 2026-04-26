package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltMotorSim;
import frc.robot.Robot;
import frc.util.SignalManager;
import frc.util.WaltLogger;

public class Intake extends SubsystemBase {
    /* CLASS VARIABLES */
    //---MOTORS + CONTROL REQUESTS
    private final TalonFX m_intakeArm = new TalonFX(kIntakeArmCANID); //x60Foc

    private final TalonFX m_intakeRollersA = new TalonFX(kIntakeRollersA_CANID); //x60Foc
    private final TalonFX m_intakeRollersB = new TalonFX(kIntakeRollersB_CANID); //x60Foc

    private MotionMagicVoltage m_MMVReq = new MotionMagicVoltage(0).withEnableFOC(true);
    private VelocityVoltage m_VelVoltReq = new VelocityVoltage(0).withEnableFOC(true);
    private VoltageOut m_voltsReq = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Current> sig_intakeArmStatorCurrent = m_intakeArm.getStatorCurrent();
    private final StatusSignal<AngularVelocity> sig_intakeArmVelo = m_intakeArm.getVelocity();
    private final StatusSignal<AngularVelocity> sig_intakeRollersAVelo = m_intakeRollersA.getVelocity();
    private final StatusSignal<Angle> sig_intakeArmPos = m_intakeArm.getPosition();
    private final StatusSignal<Boolean> sig_intakeArmMMAtTarget = m_intakeArm.getMotionMagicAtTarget();

    private BooleanSupplier m_currentSpike = () -> sig_intakeArmStatorCurrent.getValueAsDouble() > 5.0;
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(sig_intakeArmVelo.getValueAsDouble()) < 0.005;
    private BooleanSupplier m_shimmyVeloIsNearZero = () -> Math.abs(sig_intakeArmVelo.getValueAsDouble()) < 0.05;
    
    private VoltageOut m_intakeArmZeroingReq = new VoltageOut(0);

    private Debouncer m_currentDebouncer = new Debouncer(0.100, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);

    private boolean m_isIntakeArmHomed = false;

    public final BooleanSupplier intakeHomedSupp = () -> m_isIntakeArmHomed;

    /* SIM OBJECTS */
    private final DCMotorSim m_intakeArmSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            kIntakeArmMOI,
            kIntakeArmGearing
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final DCMotorSim m_intakeRollersSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(2),
            kIntakeRollersMOI,
            kIntakeRollersGearing
        ),
        DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_intakeArmRots = WaltLogger.logDouble(kLogTab, "intakeArmRots");
    private final DoubleLogger log_targetIntakeArmRots = WaltLogger.logDouble(kLogTab, "targetIntakeArmRots");

    private final DoubleLogger log_intakeRollersRPS = WaltLogger.logDouble(kLogTab, "intakeRollersRPS");
    private final DoubleLogger log_targetIntakeRollersRPS = WaltLogger.logDouble(kLogTab, "targetIntakeRollersRPS");

    private final BooleanLogger log_isIntakeArmHomed = WaltLogger.logBoolean(kLogTab, "isIntakeArmHomed");

    /* CONSTRUCTOR */
    public Intake() {
        m_intakeArm.getConfigurator().apply(kIntakeArmConfiguration);

        m_intakeRollersA.getConfigurator().apply(kIntakeRollersAConfiguration);
        m_intakeRollersB.getConfigurator().apply(kIntakeRollersBConfiguration);

        m_intakeRollersB.setControl(new Follower(kIntakeRollersA_CANID, MotorAlignmentValue.Opposed));

        SignalManager.register(kRioBus, sig_intakeArmStatorCurrent, sig_intakeArmVelo, sig_intakeRollersAVelo, sig_intakeArmPos, sig_intakeArmMMAtTarget);

        if (Robot.isReal()) {
            setDefaultCommand(intakeArmCurrentSenseHoming());
            // setDefaultCommand(intakeArmHome());
        }

        initSim();
    }

    private void initSim() {
        WaltMotorSim.initSimFX(m_intakeArm, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX44);
        WaltMotorSim.initSimFX(m_intakeRollersA, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX60);
    }

    /* COMMANDS */
    public void setIntakeArmPos(IntakeArmPosition rots) {
        setIntakeArmPos(rots.rots); // rots == IntakeArmPosition.RETRACTED ? 36 : 18
    }

    public Command setIntakeArmPosCmd(IntakeArmPosition rots) {
        return setIntakeArmPosCmd(rots.rots); // rots == IntakeArmPosition.RETRACTED ? 36 : 18
    }

    public Command setIntakeArmPosCmd(Angle rots) {
        return runOnce(() -> setIntakeArmPos(rots));
    }

    public void setIntakeArmPos(Angle rots) {
        m_intakeArm.setControl(m_MMVReq.withPosition(rots));
    }

    public boolean isIntakeArmAtDest() {
       return m_shimmyVeloIsNearZero.getAsBoolean();
    }

     public void setIntakeArmNeutralMode(NeutralModeValue value) {
        m_intakeArm.setNeutralMode(value);
    }

    public Command startIntakeRollers(double volts) {
        return setIntakeRollersVelocityCmd(volts);
    }

    public Command stopIntakeRollers() {
        return setIntakeRollersVelocityCmd(0);
    }

    public void setIntakeRollersVelocity(double volts) {
        m_intakeRollersA.setControl(m_VelVoltReq.withVelocity(volts / 12 * kIntakeRollersMaxRPS.in(RotationsPerSecond)));   ///kV = 0.488599348534
        // m_intakeRollersA.setControl(m_voltsReq.withOutput(volts));
    }

    public Command setIntakeRollersVelocityCmd(double volts) {
        return runOnce(() -> setIntakeRollersVelocity(volts));
    }

    // TESTING TO SEE IF WE CAN JUST SAY 0 AS 0
    public Command intakeArmHome() {
        return Commands.parallel(
            Commands.sequence(
                runOnce(() -> m_intakeArm.setPosition(0)),

                runOnce(() -> m_isIntakeArmHomed = true),
                runOnce(() -> log_isIntakeArmHomed.accept(m_isIntakeArmHomed)),

                runOnce(() -> removeDefaultCommand())
            )
        );
    }

    public Command intakeArmCurrentSenseHoming() {
        Runnable init = () -> {
            m_intakeArm.setControl(m_intakeArmZeroingReq.withOutput(-3.25));

            m_isIntakeArmHomed = false;
            log_isIntakeArmHomed.accept(m_isIntakeArmHomed);
        };

        Runnable execute = () -> {};

        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_intakeArm.setControl(m_intakeArmZeroingReq.withOutput(0));
            m_intakeArm.setPosition(0);
            removeDefaultCommand();
            setIntakeArmPosCmd(IntakeArmPosition.RETRACTED);
            m_isIntakeArmHomed = true;
            log_isIntakeArmHomed.accept(m_isIntakeArmHomed);
        };

        BooleanSupplier isFinished = () ->
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) &&
            m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());

        return new FunctionalCommand(init, execute, onEnd, isFinished, this).withTimeout(3).withName("intakeArm homing");
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        log_targetIntakeArmRots.accept(m_MMVReq.Position);
        log_targetIntakeRollersRPS.accept(m_voltsReq.Output);
        log_intakeRollersRPS.accept(sig_intakeRollersAVelo.getValueAsDouble());
        log_intakeArmRots.accept(sig_intakeArmPos.getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        WaltMotorSim.updateSimFX(m_intakeArm, m_intakeArmSim);
        WaltMotorSim.updateSimFX(m_intakeRollersA, m_intakeRollersSim);
    }

    /* ENUMS */
    public enum IntakeArmPosition{
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