package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.kCanivoreBus;
import static frc.robot.Constants.kShooterBus;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Hood extends SubsystemBase {
    private final TalonFXS m_hood = new TalonFXS(kHoodCANID, kShooterBus);
    private final PositionVoltage m_hoodPVRequest = new PositionVoltage(0).withEnableFOC(false);
    private final VoltageOut m_hoodZeroReq = new VoltageOut(0);

    private final BooleanLogger log_hoodHomed = WaltLogger.logBoolean("Shooter/Hood", "Homed");
    private final DoubleLogger log_hoodControlPos = WaltLogger.logDouble("Shooter/Hood", "hoodControlPos");
    private final DoubleLogger log_hoodCurrentPos = WaltLogger.logDouble("Shooter/Hood", "hoodCurrentPos");

    private Debouncer m_currentDebouncer = new Debouncer(0.100, DebounceType.kRising);
    private Debouncer m_velocityDebouncer = new Debouncer(0.125, DebounceType.kRising);

    private BooleanSupplier m_currentSpike = () -> m_hood.getStatorCurrent().getValueAsDouble() > 10.0;
    private BooleanSupplier m_veloIsNearZero = () -> Math.abs(m_hood.getVelocity().getValueAsDouble()) < 0.005;

    private final StaticBrake m_BrakeReq = new StaticBrake();

    private boolean m_isHoodHomed = false;

    public Hood() {
        m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        setDefaultCommand(hoodCurrentSenseHomingCmd());
    }

    // ---HOOD
    public void setHoodPos(Angle degs) {
        m_hood.setControl(m_hoodPVRequest.withPosition(degs));
        log_hoodControlPos.accept(degs.in(Degrees));
    }

    public void setHoodPos(double rots) {
        m_hood.setControl(m_hoodPVRequest.withPosition(rots));
        log_hoodControlPos.accept(rots * 360.0);
    }

    public Command setHoodPosCmd(Angle degs) {
        return runOnce(() -> setHoodPos(degs));
    }

    //for TestingDashboard
    public Command setHoodPositionCmd(DoubleSubscriber sub_degs) {
        return run(() -> setHoodPos(Degrees.of(sub_degs.get())));
    }

    private Angle getHoodAngle() {
        return m_hood.getPosition().getValue();
    }
    
    public boolean isHoodHomed() {
        return m_isHoodHomed;
    }

    public Command hoodCurrentSenseHomingCmd(){
        Runnable init = () -> {
            m_hood.setControl(m_hoodZeroReq.withOutput(kHomingVoltage));
            m_isHoodHomed = false;
            log_hoodHomed.accept(m_isHoodHomed);
        };

        Consumer<Boolean> end = (Boolean interrupted) -> {
            if (interrupted) {
                m_hood.setControl(m_BrakeReq);
                WaltLogger.timedPrint("HoodHoming INTERRUPTED!!!!");
                log_hoodHomed.accept(m_isHoodHomed);
                return;
            }

            m_hood.setPosition(kHoodMinPosition);
            m_hood.setControl(m_BrakeReq);
            removeDefaultCommand();
            m_isHoodHomed = true;
            log_hoodHomed.accept(m_isHoodHomed);
        };

        BooleanSupplier isFinished = () -> 
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()) &&
            m_velocityDebouncer.calculate(m_veloIsNearZero.getAsBoolean());

        return new FunctionalCommand(init, () -> {}, end, isFinished, this).withTimeout(5);
    }

    public void setHoodNeutralMode(NeutralModeValue value) {
        m_hood.setNeutralMode(value);
    }

    @Override
    public void periodic() {
        // log_hoodCurrentPos.accept(getHoodAngle().in(Degrees)); 
    }
}
