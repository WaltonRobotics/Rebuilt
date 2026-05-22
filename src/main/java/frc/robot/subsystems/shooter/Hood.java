package frc.robot.subsystems.shooter;

import static frc.robot.Constants.kShooterBus;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
import frc.util.SignalManager;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

/*
 * Hood - TalonFXS-driven hood angle control.
 * Zero is set at the hardstop via current-sense homing. Position is in rotations.
 *
 * Vocab:
 *   homing - drives to the hardstop at low voltage, detects stall via stator current spike through a Debouncer
 *   atPos - closed-loop error is within kHoodMaxErrD of the setpoint
 */
public class Hood extends SubsystemBase {
    // ---- CONSTANTS ----
    private static final String kLogTab = "Shooter/Hood";
    // converts encoder rotations to physical hood angle degrees for logging
    private static final double kAbsoluteToPhysicalAngleRatio =
        (kPhysicalHoodMaxPosition_double - kPhysicalHoodMinPosition_double) / (360.0 * (kHoodMaxRots_double - kHoodMinRots_double));

    // ---- MOTOR + CONTROLS ----
    private final TalonFXS m_hood = new TalonFXS(kHoodCANID, kShooterBus);
    private final PositionVoltage m_hoodPVRequest = new PositionVoltage(0).withEnableFOC(false);
    private final VoltageOut m_hoodZeroReq = new VoltageOut(0);
    private final StaticBrake m_BrakeReq = new StaticBrake();

    // ---- SIGNALS ----
    private final StatusSignal<Current> sig_hoodStatorCurrent = m_hood.getStatorCurrent();
    private final StatusSignal<Angle> sig_hoodPos = m_hood.getPosition();
    private final StatusSignal<Double> sig_hoodCLErr = m_hood.getClosedLoopError();

    // ---- LOGGERS ----
    private final BooleanLogger log_hoodHomed = WaltLogger.logBoolean(kLogTab, "Homed");
    private final DoubleLogger log_hoodControlPos = WaltLogger.logDouble(kLogTab, "controlPos");
    private final DoubleLogger log_hoodCurrentPos = WaltLogger.logDouble(kLogTab, "currentPos");
    private final DoubleLogger log_hoodPositionDeg = WaltLogger.logDouble(kLogTab, "positionDeg"); //for debugging purposes
    private final DoubleLogger log_hoodCLErr = WaltLogger.logDouble(kLogTab, "closedLoopErr");
    private final BooleanLogger log_hoodAtPos = WaltLogger.logBoolean(kLogTab, "hoodAtPos");

    // ---- STATE ----
    private Debouncer m_currentDebouncer = new Debouncer(0.125, DebounceType.kRising);
    private BooleanSupplier m_currentSpike = () -> sig_hoodStatorCurrent.getValueAsDouble() > 5.0;
    private boolean m_isHoodHomed = false;
    private boolean m_hoodAtPos = false;

    // =============================================================
    // CONSTRUCTOR
    // =============================================================

    public Hood() {
        m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        SignalManager.register(kShooterBus, sig_hoodStatorCurrent, sig_hoodPos, sig_hoodCLErr);

        m_hood.setPosition(0);
        m_isHoodHomed = true;
        log_hoodHomed.accept(m_isHoodHomed);
        setHoodPos(0.05); //really really low position to see that this is working

        // setDefaultCommand(hoodCurrentSenseHomingCmd());
    }

    // =============================================================
    // CONTROL
    // =============================================================

    public void setHoodPos(double rots) {
        m_hood.setControl(m_hoodPVRequest.withPosition(rots));
        log_hoodControlPos.accept(rots);
    }

    public Command setHoodPosCmd(double rots) {
        return runOnce(() -> setHoodPos(rots));
    }

    // converts encoder rotations to physical degrees - only used for logging
    private static double getHoodAngleDeg(double posRots) {
        return kPhysicalHoodMinPosition_double + (posRots * 360.0 - kHoodMinRots_double * 360.0) * kAbsoluteToPhysicalAngleRatio;
    }

    public void setHoodNeutralMode(NeutralModeValue value) {
        m_hood.setNeutralMode(value);
    }

    // =============================================================
    // STATE
    // =============================================================

    public boolean isHoodHomed() {
        return m_isHoodHomed;
    }

    private void refreshHoodAtPos() {
        log_hoodCLErr.accept(sig_hoodCLErr.getValueAsDouble());
        m_hoodAtPos = sig_hoodPos.isNear(0, kHoodMaxErrD);
        log_hoodAtPos.accept(m_hoodAtPos);
    }

    public boolean atPosition() {
        return m_hoodAtPos;
    }

    // =============================================================
    // HOMING
    // =============================================================

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

    // =============================================================
    // PERIODIC
    // =============================================================

    @Override
    public void periodic() {
        double posRots = sig_hoodPos.getValueAsDouble();
        refreshHoodAtPos();
        log_hoodCurrentPos.accept(getHoodAngleDeg(posRots));
        log_hoodPositionDeg.accept(posRots * 360.0);
    }
}
