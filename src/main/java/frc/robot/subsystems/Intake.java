package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Intake extends SubsystemBase {
    // TODO add the necessary ports
    private final TalonFX m_intakeDeployMotor = new TalonFX(1); // TODO put IDs
    private final TalonFX m_intakeRollerMotor = new TalonFX(1);
    
    private MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    private VoltageOut m_rollerVoltOut = new VoltageOut(/*put value here */);

    private VoltageOut zeroingVoltageCtrlReq = new VoltageOut(-0.75); // TODO change this to whatever necessary

    private BooleanSupplier m_currentSpike = () -> m_intakeDeployMotor.getStatorCurrent().getValueAsDouble() > 5.0;  // TODO adjust these two as necessary
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(m_intakeDeployMotor.getVelocity().getValueAsDouble()) < 0.005;

    private Debouncer m_currentDebouncer = new Debouncer(0.25, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);

    private boolean m_zeroed = false;

    
    public Intake (){
        // m_intakeDeployMotor.getConfigurator().apply();
        // m_intakeRollerMotor.getConfigurator().apply();
        // TODO fix the above
    }

    public Command spinIntakeRollers(){
        return Commands.runOnce(() -> m_intakeRollerMotor.setVoltage(/* put voltage here */));
    }

    public Command intakeToAngle(double degs){
        return runOnce(
            () -> {
                m_MMVRequest = m_MMVRequest.withPosition(degs);
                var request = m_MMVRequest;
                m_intakeDeployMotor.setControl(request);
            }
        );
    }

    public Command resetAngle(){
        Runnable init = () -> {
            m_intakeDeployMotor.setControl(zeroingVoltageCtrlReq.withOutput(-1));
        };
        Runnable execute = () -> {};
        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_intakeDeployMotor.setPosition(0);
            m_intakeDeployMotor.setControl(zeroingVoltageCtrlReq.withOutput(0));
            removeDefaultCommand();
            m_zeroed = true;
        };

        BooleanSupplier isFinished = () ->
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) && 
            m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());

        return new FunctionalCommand(init, execute, onEnd, isFinished, this);
    }
}
