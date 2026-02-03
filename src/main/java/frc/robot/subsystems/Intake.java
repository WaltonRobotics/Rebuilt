package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.RobotController;
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

    private MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0);
    private VoltageOut m_rollerVoltOut = new VoltageOut(12);

    private VoltageOut zeroingVoltageCtrlReq = new VoltageOut(-0.75); // TODO change this to whatever necessary

    private boolean m_zeroed = false;

    // Loggers
    BooleanLogger log_isWorking = WaltLogger.logBoolean(IntakeK.kLogTab, "working");
    DoubleLogger log_rollerVoltOut = WaltLogger.logDouble(IntakeK.kLogTab, "rollerVoltOut");
    DoubleLogger log_rollerSpeed = WaltLogger.logDouble(IntakeK.kLogTab, "rollerSpeedt");
    DoubleLogger log_deployPosition = WaltLogger.logDouble(IntakeK.kLogTab, "deployPos");

    // Simulators
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
        var deployFXSim = m_intakeDeployMotor.getSimState();
        deployFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        deployFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        var rollerFXSim = m_intakeDeployMotor.getSimState();
        rollerFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        rollerFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    public Command spinIntakeRollers() {
        return runOnce(() -> m_intakeRollerMotor.setVoltage(m_rollerVoltOut.Output));
    }

    public Command intakeToAngle(double rots) {
        return runOnce(
            () -> m_intakeDeployMotor.setControl(m_MMVRequest.withPosition(rots))
        );
    }

    @Override
    public void periodic() {
        log_rollerVoltOut.accept(m_MMVRequest.Position);
        log_rollerSpeed.accept(m_intakeRollerMotor.getVelocity().getValueAsDouble());
        log_deployPosition.accept(m_intakeDeployMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var rollerFXSim = m_intakeRollerMotor.getSimState();

        m_rollerSim.setInputVoltage(rollerFXSim.getMotorVoltage());
        m_rollerSim.update(0.02);
        
        rollerFXSim.setRawRotorPosition(m_rollerSim.getAngularPositionRotations());
        rollerFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}