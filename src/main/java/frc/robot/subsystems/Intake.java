package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.StringLogger;


import frc.robot.Constants.IntakeK;
import frc.util.WaltLogger;

public class Intake extends SubsystemBase {
    private final TalonFX m_intakeDeployMotor = new TalonFX(IntakeK.kIntakeDeployCANID);
    private final TalonFX m_intakeRollerMotor = new TalonFX(IntakeK.kIntakeRollerCANID);

    private MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    private VoltageOut m_rollerVoltOut = new VoltageOut(1);

    private VoltageOut zeroingVoltageCtrlReq = new VoltageOut(-0.75); // TODO change this to whatever necessary

    private CommandXboxController m_controller = new CommandXboxController(0);

    // TODO possibly uncomment the lines below this if necessary
    // private BooleanSupplier m_currentSpike = () ->
    // m_intakeDeployMotor.getStatorCurrent().getValueAsDouble() > 5.0; // TODO
    // adjust these two as necessary
    // private BooleanSupplier m_veloIsNearZero = () ->
    // Math.abs(m_intakeDeployMotor.getVelocity().getValueAsDouble()) < 0.005;

    // private Debouncer m_currentDebouncer = new Debouncer(0.25,
    // DebounceType.kRising);
    // private Debouncer m_velocityDebouncer = new Debouncer(0.125,
    // DebounceType.kRising);

    private boolean m_zeroed = false;

    // Logger stuff
    BooleanLogger log_isWorking = WaltLogger.logBoolean(IntakeK.kLogTab, "working");
    DoubleLogger log_getRollerVoltOut = WaltLogger.logDouble(IntakeK.kLogTab, "rollerVoltOut");
    DoubleLogger log_deployPosition = WaltLogger.logDouble(IntakeK.kLogTab, "deployPos");

    private final DCMotorSim m_deploySim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1),
            0.0005,  // Dummy J value
            1.5 // dummy gearing value
        ),
        DCMotor.getKrakenX44(1) // returns gearbox
    );

    private final DCMotorSim m_rollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1),
            0.0005,  // Dummy J value
            1.5 // dummy gearing value
        ),
        DCMotor.getKrakenX44(1) // returns gearbox
    );

    public Intake() {
        m_intakeDeployMotor.getConfigurator().apply(IntakeK.kDeployConfiguration);
        m_intakeRollerMotor.getConfigurator().apply(IntakeK.kIntakeConfiguration);
        initSim();
    }

    private void initSim() {
        var m_deployFXSim = m_intakeDeployMotor.getSimState();
        m_deployFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        m_deployFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        var m_rollerFXSim = m_intakeDeployMotor.getSimState();
        m_rollerFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        m_rollerFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    public Command spinIntakeRollers() {
        return Commands.runOnce(() -> m_intakeRollerMotor.setVoltage(m_rollerVoltOut.Output));
    }

    public Command intakeToAngle(double degs) {
        return runOnce(
            () -> {
                m_intakeDeployMotor.setControl(m_MMVRequest.withPosition(degs));
            });
    }

    public Command resetAngle() {
        Runnable init = () -> {
            m_intakeDeployMotor.setControl(zeroingVoltageCtrlReq.withOutput(-1));
        };
        Runnable execute = () -> {
        };
        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_intakeDeployMotor.setPosition(0);
            m_intakeDeployMotor.setControl(zeroingVoltageCtrlReq.withOutput(0));
            removeDefaultCommand();
            m_zeroed = true;
        };
        BooleanSupplier isFinished = () -> true;
        // BooleanSupplier isFinished = () ->
        // m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) &&
        // m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());
        // TODO possibly add this not sure if it's applicable to the REBUILT robot.
        return new FunctionalCommand(init, execute, onEnd, isFinished, this);
    }

    public void simulationPeriodic(){
        // m_controller.a().onTrue(intakeToAngle(0));
        // m_controller.b().onTrue(intakeToAngle(0.5));
        // m_controller.x().onTrue(intakeToAngle(1));
        log_isWorking.accept(true);
        spinIntakeRollers();
        log_getRollerVoltOut.accept(m_intakeRollerMotor.getSupplyVoltage().getValueAsDouble());
        // log_deployPosition.accept(m_intakeDeployMotor);
    }
}