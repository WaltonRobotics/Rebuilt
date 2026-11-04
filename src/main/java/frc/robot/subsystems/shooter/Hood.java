package frc.robot.subsystems.shooter;

import static frc.robot.Constants.kShooterBus;
import static frc.robot.Constants.ShooterK.*;
import static org.wpilib.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

import frc.robot.Constants.ShooterK;
import frc.util.SignalManager;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Hood implements Mechanism {
    private static final String kLogTab = "Shooter/Hood";
    private static final double kAbsoluteToPhysicalAngleRatio =
        (kPhysicalHoodMaxPosition_double - kPhysicalHoodMinPosition_double) / (360.0 * (kHoodMaxRots_double - kHoodMinRots_double));
    private final TalonFXS m_hood = new TalonFXS(kHoodCANID, kShooterBus);
    private final PositionVoltage m_hoodPVRequest = new PositionVoltage(0).withEnableFOC(false);
    private final VoltageOut m_hoodZeroReq = new VoltageOut(0);

    private final BooleanLogger log_hoodHomed = WaltLogger.logBoolean(kLogTab, "Homed");
    private final DoubleLogger log_hoodControlPos = WaltLogger.logDouble(kLogTab, "controlPos");
    private final DoubleLogger log_hoodCurrentPos = WaltLogger.logDouble(kLogTab, "currentPos");
    private final DoubleLogger log_hoodPositionDeg = WaltLogger.logDouble(kLogTab, "positionDeg"); //for debugging purposes
    private final DoubleLogger log_hoodCLErr = WaltLogger.logDouble(kLogTab, "closedLoopErr");
    private final BooleanLogger log_hoodAtPos = WaltLogger.logBoolean(kLogTab, "hoodAtPos");
    private final BooleanLogger log_hoodStatorCurrentCheck = WaltLogger.logBoolean(kLogTab, "statorCurrentFlag");

    private Debouncer m_currentDebouncer = new Debouncer(0.125, DebounceType.RISING);

    private final StatusSignal<Current> sig_hoodStatorCurrent = m_hood.getStatorCurrent();
    private final StatusSignal<Angle> sig_hoodPos = m_hood.getPosition();
    private final StatusSignal<Double> sig_hoodCLErr = m_hood.getClosedLoopError();

    private BooleanSupplier m_currentSpike = () -> sig_hoodStatorCurrent.getValueAsDouble() > 3.0;

    private final StaticBrake m_BrakeReq = new StaticBrake();

    private boolean m_isHoodHomed = false;
    private boolean m_hoodAtPos = false;
    private double m_commandedPosRots = 0.0;

    public Hood() {
        m_hood.getConfigurator().apply(kHoodTalonFXSConfiguration);

        SignalManager.register(kShooterBus, sig_hoodStatorCurrent, sig_hoodPos, sig_hoodCLErr);

        m_hood.setPosition(0);
        m_isHoodHomed = true;
        log_hoodHomed.accept(m_isHoodHomed);
        setHoodPos(0.05); //really really low position to see that this is working

        Scheduler.getDefault().addPeriodic(this::periodic);

        // setDefaultCommand(hoodCurrentSenseHomingCmd());
    }

    // ---HOOD
    public void setHoodPos(double rots) {
        m_commandedPosRots = rots;
        m_hood.setControl(m_hoodPVRequest.withPosition(rots));
        log_hoodControlPos.accept(rots);
    }

    public Command setHoodPosCmd(double rots) {
        // return runOnce(() -> setHoodPos(rots));

        return run(coroutine -> {
            setHoodPos(rots);
        }).named("setHoodPosCmd");
    }

    private static double getHoodAngleDeg(double posRots) {
        return kPhysicalHoodMinPosition_double + (posRots * 360.0 - kHoodMinRots_double * 360.0) * kAbsoluteToPhysicalAngleRatio;
    }

    public boolean isHoodHomed() {
        return m_isHoodHomed;
    }

    private void refreshHoodAtPos() {
        log_hoodCLErr.accept(sig_hoodCLErr.getValueAsDouble());
        // m_hoodAtPos = Math.abs(sig_hoodPos.getValueAsDouble() - m_commandedPosRots) <= kHoodMaxErrD;
        m_hoodAtPos = sig_hoodCLErr.getValueAsDouble() <= kHoodMaxErrD; // all in rotations
        log_hoodAtPos.accept(m_hoodAtPos);
    }

    public boolean atPosition() {
        return m_hoodAtPos;
    }

    /**
     * home the hood by sensing the current idk man<br>
     * 
     * NOTE: removeDefaultCommand, now known as setDefaultCommand(idle()), was not implemented because the default
     * command in the constructor is commented out. in the future, if it is uncommented, keep in mind that running
     * setDefaultCommand in a Command makes it only the default command for as long as the command is active.
     * afterwards, it reverts back to the previous default command.
     * 
     * @see org.wpilib.command3.Scheduler#setDefaultCommand()
     */
    public Command hoodCurrentSenseHomingCmd(){
        // Runnable init = () -> {
        //     m_hood.getConfigurator().apply(ShooterK.kHoodTalonFXSConfigurationNoSoftLimit);
        //     m_hood.setControl(m_hoodZeroReq.withOutput(kHoodHomingVoltage));
        //     m_isHoodHomed = false;
        //     log_hoodHomed.accept(m_isHoodHomed);
        // };

        // Consumer<Boolean> end = (Boolean interrupted) -> {
        //     if (interrupted) {
        //         m_hood.setControl(m_BrakeReq);
        //         WaltLogger.timedPrint("HoodHoming INTERRUPTED!!!!");
        //         log_hoodHomed.accept(m_isHoodHomed);
        //         return;
        //     }

        //     m_hood.setPosition(kHoodAbsoluteMinRots);
        //     m_hood.setControl(m_BrakeReq);
        //     removeDefaultCommand();
        //     m_isHoodHomed = true;
        //     m_hood.getConfigurator().apply(ShooterK.kHoodTalonFXSConfiguration);
        //     log_hoodHomed.accept(m_isHoodHomed);
        // };

        // BooleanSupplier isFinished = () -> 
        //     m_currentDebouncer.calculate(m_currentSpike.getAsBoolean());

        // return new FunctionalCommand(init, () -> {}, end, isFinished, this).withTimeout(5);

        return run(coroutine -> {
            m_hood.getConfigurator().apply(ShooterK.kHoodTalonFXSConfigurationNoSoftLimit);
            m_hood.setControl(m_hoodZeroReq.withOutput(kHoodHomingVoltage));
            m_isHoodHomed = false;
            log_hoodHomed.accept(m_isHoodHomed);

            coroutine.waitUntil(() -> m_currentDebouncer.calculate(m_currentSpike.getAsBoolean()));

            m_hood.setPosition(kHoodAbsoluteMinRots); // Success code starts here, if it fails command goes to whenCanceled
            m_hood.setControl(m_BrakeReq);
            m_isHoodHomed = true;
            m_hood.getConfigurator().apply(ShooterK.kHoodTalonFXSConfiguration);
            log_hoodHomed.accept(m_isHoodHomed);
        }).whenCanceled(() -> {
            m_hood.setControl(m_BrakeReq);
            WaltLogger.timedPrint("HoodHoming INTERRUPTED!!!!");
            log_hoodHomed.accept(m_isHoodHomed);
        }).named("hoodCurrentSenseHomingCmd").withTimeout(Seconds.of(5));
    }

    public void setHoodNeutralMode(NeutralModeValue value) {
        // m_hood.setNeutralMode(value);
    }

    public void periodic() {
        double posRots = sig_hoodPos.getValueAsDouble();
        refreshHoodAtPos();
        log_hoodStatorCurrentCheck.accept(m_currentSpike);
        log_hoodCurrentPos.accept(getHoodAngleDeg(posRots));
        log_hoodPositionDeg.accept(posRots * 360.0);
    }
}
