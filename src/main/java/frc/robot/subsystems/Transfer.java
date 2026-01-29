package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
    private final TalonFX m_transferSpinnerMotor = new TalonFX(1); // change IDs
    private final TalonFX m_transferExhaustMotor = new TalonFX(1);

    public Transfer() {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = 0.1;
        slot0.kV = 0.12;
        slot0.kP = 0.11;
        slot0.kI = 0;                       // adjust values for more accuracy later
        slot0.kD = 0;

        m_transferSpinnerMotor.getConfigurator().apply(slot0);
        m_transferExhaustMotor.getConfigurator().apply(slot0);
    }

    public void startSpinner(double rpm) {
        m_transferSpinnerMotor.setControl(new VelocityDutyCycle(rpm / 60));
    }

    public void stopSpinner() {
        m_transferSpinnerMotor.setControl(new VelocityDutyCycle(0));
    }

    public void startExhaust(double rpm) {
        m_transferExhaustMotor.setControl(new VelocityDutyCycle(rpm / 60));
    }

    public void stopExhaust() {
        m_transferExhaustMotor.setControl(new VelocityDutyCycle(0));
    }
}
