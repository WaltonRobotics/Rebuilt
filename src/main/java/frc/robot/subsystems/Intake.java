package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IntakeK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltMotorSim;
import frc.robot.Robot;
import frc.util.WaltLogger;

public class Intake extends SubsystemBase {
    /* CLASS VARIABLES */
    //---MOTORS + CONTROL REQUESTS
    private final TalonFX m_intakeArm = new TalonFX(kIntakeArmCANID); //x44Foc
    private final TalonFX m_intakeRollers = new TalonFX(kIntakeRollersCANID); //x60Foc

    private DynamicMotionMagicVoltage m_MMVReq = new DynamicMotionMagicVoltage(0, 1, 1).withEnableFOC(true);
    private VelocityVoltage m_VVReq = new VelocityVoltage(0).withEnableFOC(true);

    private BooleanSupplier m_currentSpike = () -> m_intakeArm.getStatorCurrent().getValueAsDouble() > 5.0;
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(m_intakeArm.getVelocity().getValueAsDouble()) < 0.005;

    private VoltageOut m_intakeArmZeroingReq = new VoltageOut(0);

    private Debouncer m_currentDebouncer = new Debouncer(0.100, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);

    private boolean m_isIntakeArmHomed = false;
    public final BooleanSupplier intakeHomedSupp = () -> m_isIntakeArmHomed;

    /* SIM OBJECTS */
    private final DCMotorSim m_intakeArmSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1),
            kIntakeArmMOI,
            kIntakeArmGearing
        ),
        DCMotor.getKrakenX44Foc(1)
    );

    private final DCMotorSim m_intakeRollersSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            kIntakeRollersMOI,
            kIntakeRollersGearing
        ),
        DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_intakeArmRots = WaltLogger.logDouble(kLogTab, "intakeArmRots");
    private final DoubleLogger log_targetIntakeArmRots = WaltLogger.logDouble(kLogTab, "targetIntakeArmRots");
    private final DoubleLogger log_intakeArmClosedLoopError = WaltLogger.logDouble(kLogTab, "intakeArmClosedLoopError");
    
    private final DoubleLogger log_intakeRollersRPS = WaltLogger.logDouble(kLogTab, "intakeRollersRPS");
    private final DoubleLogger log_targetIntakeRollersRPS = WaltLogger.logDouble(kLogTab, "targetIntakeRollersRPS");

    private final BooleanLogger log_isIntakeArmHomed = WaltLogger.logBoolean(kLogTab, "isIntakeArmHomed");

    /* CONSTRUCTOR */
    public Intake() {
        m_intakeArm.getConfigurator().apply(kIntakeArmConfiguration);
        m_intakeRollers.getConfigurator().apply(kIntakeRollersConfiguration);

        if (Robot.isReal()) {
            setDefaultCommand(intakeArmCurrentSenseHoming());
        }

        initSim();
    }

    private void initSim() {
        WaltMotorSim.initSimFX(m_intakeArm, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX44);
        WaltMotorSim.initSimFX(m_intakeRollers, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX60);
    }

    /* COMMANDS */
    public void setIntakeArmPos(IntakeArmPosition rots) {
        setIntakeArmPos(rots.rots, rots == IntakeArmPosition.RETRACTED ? 4 : 2);
    }

    public Command setIntakeArmPosCmd(IntakeArmPosition rots) {
        return setIntakeArmPosCmd(rots.rots, rots == IntakeArmPosition.RETRACTED ? 4 : 2);
    }

    public Command setIntakeArmPosCmd(Angle rots, double RPSPS) {
        return runOnce(() -> setIntakeArmPos(rots, RPSPS));
    }

    public void setIntakeArmPos(Angle rots, double RPSPS) {
        m_intakeArm.setControl(m_MMVReq.withPosition(rots).withAcceleration(RPSPS));
    }

    public boolean isIntakeArmAtPos() {
        var err = m_intakeArm.getClosedLoopError();
        log_intakeArmClosedLoopError.accept(err.getValueAsDouble());
        boolean isNear = m_intakeArm.getClosedLoopError().isNear(0, 0.01);
        return isNear;
    }

    public Command shimmy() {
        return Commands.repeatingSequence(
            setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            setIntakeRollersVelocityCmd(RotationsPerSecond.zero()),
            setIntakeArmPosCmd(IntakeArmPosition.SHIMMY),
            Commands.waitUntil(() -> isIntakeArmAtPos()),
            setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> isIntakeArmAtPos())
        ).finallyDo(() -> setIntakeArmPosCmd(IntakeArmPosition.SAFE));
    }

     public void setIntakeArmNeutralMode(NeutralModeValue value) {
        m_intakeArm.setNeutralMode(value);
    }

    //for TestingDashboard
    public Command setIntakeArmPos(DoubleSubscriber sub_rots) {
        return run(() -> m_intakeArm.setControl(m_MMVReq.withPosition(Rotations.of(sub_rots.get()))));
    }

    public Command startIntakeRollers() {
        return setIntakeRollersVelocityCmd(kIntakeRollersMaxRPS);
    }

    public Command stopIntakeRollers() {
        return setIntakeRollersVelocityCmd(RotationsPerSecond.zero());
    }

    public void setIntakeRollersVelocity(AngularVelocity RPS) {
        m_intakeRollers.setControl(m_VVReq.withVelocity(RPS));
    }

    public Command setIntakeRollersVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_intakeRollers.setControl(m_VVReq.withVelocity(RPS)));
    }

    //for TestingDashboard
    public Command setIntakeRollersVelocity(DoubleSubscriber sub_RPS) {
        return run(() -> m_intakeRollers.setControl(m_VVReq.withVelocity(RotationsPerSecond.of(sub_RPS.get()))));
    }

    // TESTING TO SEE IF WE CAN JUST SAY 0 AS 0
    // public Command intakeArmHome() {
    //     return Commands.sequence(
    //         runOnce(() -> m_intakeArm.setPosition(0)),
    //         runOnce(() -> m_isIntakeArmHomed = true),
    //         runOnce(() -> log_isIntakeArmHomed.accept(m_isIntakeArmHomed)),
    //         runOnce(() -> removeDefaultCommand())
    //     );
    // }

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
        // log_targetIntakeArmRots.accept(m_PVReq.Position);
        log_targetIntakeRollersRPS.accept(m_VVReq.Velocity);
        log_intakeRollersRPS.accept(m_intakeRollers.getVelocity().getValueAsDouble());
        log_intakeArmRots.accept(m_intakeArm.getPosition().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        WaltMotorSim.updateSimFX(m_intakeArm, m_intakeArmSim);
        WaltMotorSim.updateSimFX(m_intakeRollers, m_intakeRollersSim);
    }

    /* ENUMS */
    public enum IntakeArmPosition{
        RETRACTED(Rotations.of(0.071514).in(Degrees)),
        DEPLOYED(Rotations.of(0.289062 - (0.025)).in(Degrees)),
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