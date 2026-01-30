package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TransferK.*;

public class Transfer extends SubsystemBase {
    private final TalonFX m_transferSpinnerMotor = new TalonFX(kSpinnerCANID);
    private final TalonFX m_transferExhaustMotor = new TalonFX(kExhaustCANID);

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
}
