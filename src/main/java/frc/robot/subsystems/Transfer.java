package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TransferK.*;

public class Transfer extends SubsystemBase {
    private final TalonFX m_transferSpinnerMotor = new TalonFX(kSpinnerCANID);
    private final TalonFX m_transferExhaustMotor = new TalonFX(kExhaustCANID);

    private final DCMotorSim m_spinnerMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(kSpinnerSlot0Configs.kV, kSpinnerSlot0Configs.kA),
         null,                                                 //TODO: add gearbox and stddev values
          null);

    private final DCMotorSim m_exhaustMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1)),
         null);                                              //TODO: add gearbox and stddev values

    public Transfer() {
        m_transferSpinnerMotor.getConfigurator().apply(kSpinnerSlot0Configs);
        m_transferExhaustMotor.getConfigurator().apply(kExhaustSlot1Configs);
    }

    public void startSpinner(double rpm) {
        m_transferSpinnerMotor.setControl(new VelocityVoltage(rpm / 60));
    }

    public void stopSpinner() {
        m_transferSpinnerMotor.setControl(new VelocityVoltage(0));
    }

    public void startExhaust(double rpm) {
        m_transferExhaustMotor.setControl(new VelocityVoltage(rpm / 60));
    }

    public void stopExhaust() {
        m_transferExhaustMotor.setControl(new VelocityVoltage(0));
    }

    public void simulationInit() {
        var m_transferSpinnerMotorSim = m_transferSpinnerMotor.getSimState();
        var m_transferExhaustMotorSim = m_transferExhaustMotor.getSimState();
        m_transferSpinnerMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        m_transferExhaustMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public void simulationPeriodic() {
        var m_transferSpinnerMotorSim = m_transferSpinnerMotor.getSimState();
        var m_transferExhaustMotorSim = m_transferExhaustMotor.getSimState();
        
    }
}
