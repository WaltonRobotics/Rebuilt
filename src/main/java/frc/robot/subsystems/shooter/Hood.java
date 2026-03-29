package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
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
    private final DoubleLogger log_hoodPositionDeg = WaltLogger.logDouble("Shooter/Hood", "hoodPositionDeg"); //for debugging purposes

    private Debouncer m_currentDebouncer = new Debouncer(0.125, DebounceType.kRising);

    private BooleanSupplier m_currentSpike = () -> m_hood.getStatorCurrent().getValueAsDouble() > 5.0;

    private final StaticBrake m_BrakeReq = new StaticBrake();

    private boolean m_isHoodHomed = false;

    public Hood() {
        m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        m_hood.setPosition(0);
        m_isHoodHomed = true;
        log_hoodHomed.accept(m_isHoodHomed);

        // setDefaultCommand(hoodCurrentSenseHomingCmd());
    }

    // ---HOOD
    public void setHoodPos(double rots) {
        m_hood.setControl(m_hoodPVRequest.withPosition(rots));
        log_hoodControlPos.accept(rots);
    }

    public Command setHoodPosCmd(double rots) {
        return runOnce(() -> setHoodPos(rots));
    }

    //for TestingDashboard
    public Command setHoodPositionCmd(DoubleSubscriber sub_rots) {
        return run(() -> setHoodPos(sub_rots.get()));
    }

    private double getHoodAngleDeg() {
        double hoodPositionDeg = m_hood.getPosition().getValue().in(Degrees);
        double absoluteToPhysicalAngleRatio = (360 * (kHoodMaxRots_double - kHoodMinRots_double))/(kPhysicalHoodMaxPosition_double - kPhysicalHoodMinPosition_double);

        return kPhysicalHoodMinPosition_double + (hoodPositionDeg - (kHoodMinRots_double * 360)) * (absoluteToPhysicalAngleRatio);
    }

    public boolean isHoodHomed() {
        return m_isHoodHomed;
    }

    public Command hoodCurrentSenseHomingCmd(){
        Runnable init = () -> {
            m_hood.getConfigurator().apply(ShooterK.kHoodTalonFXSConfigurationNoSoftLimit);
            m_hood.setControl(m_hoodZeroReq.withOutput(kHoodHomingVoltage));
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

            m_hood.setPosition(kHoodAbsoluteMinRots);
            m_hood.setControl(m_BrakeReq);
            removeDefaultCommand();
            m_isHoodHomed = true;
            m_hood.getConfigurator().apply(ShooterK.kHoodTalonFXSConfiguration);
            log_hoodHomed.accept(m_isHoodHomed);
        };

        BooleanSupplier isFinished = () -> 
            m_currentDebouncer.calculate(m_currentSpike.getAsBoolean());

        return new FunctionalCommand(init, () -> {}, end, isFinished, this).withTimeout(5);
    }

    public void setHoodNeutralMode(NeutralModeValue value) {
        m_hood.setNeutralMode(value);
    }

    @Override
    public void periodic() {
        log_hoodCurrentPos.accept(getHoodAngleDeg());
        log_hoodPositionDeg.accept(m_hood.getPosition().getValue().in(Degrees));
    }
}
