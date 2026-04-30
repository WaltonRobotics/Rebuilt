package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IndexerK.*;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.util.SignalManager;
import frc.util.WaltMotorSim;
import frc.util.WaltTunable;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Indexer extends SubsystemBase {
    /* CLASS VARIABLES */
    //---MOTORS + CONTROL REQUESTS
    private final TalonFX m_spindexer = new TalonFX(kSpindexerCANID, Constants.kCanivoreBus); // X60Foc
    private final TalonFX m_tunnel = new TalonFX(kTunnelCANID, Constants.kCanivoreBus); // X60Foc

    private final VelocityVoltage m_spindexerVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage m_tunnelVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private static final WaltTunable kTunnelRPSOverride = new WaltTunable("/Indexer/Tunnel/tunnelRPSOverride", kTunnelShootRPSD);
    private static final WaltTunable kSpindexerRPSOverride = new WaltTunable("/Indexer/Spindexer/spindexerRPSOverride", kSpindexerShootRPSD);

    private final CoastOut m_spindexerMotorIdleReq = new CoastOut();
    private final CoastOut m_tunnelMotorIdleReq = new CoastOut();

    /* SIM OBJECTS */
    // private final DCMotorSim m_spindexerSim = new DCMotorSim(
    //     LinearSystemId.createDCMotorSystem(
    //         DCMotor.getKrakenX60Foc(1),
    //         kSpindexerMOI,
    //         kSpindexerGearing
    //     ),
    //     DCMotor.getKrakenX60Foc(1)
    // );

    // private final DCMotorSim m_tunnelSim = new DCMotorSim(
    //     LinearSystemId.createDCMotorSystem(
    //         DCMotor.getKrakenX60Foc(1),
    //         kTunnelMOI,
    //         kTunnelGearing
    //     ),
    //     DCMotor.getKrakenX60Foc(1)
    // );

    
    /* TUNABLES */
    private static final WaltTunable kTunnelRatioMultiplier = new WaltTunable("/Indexer/Tunnel/tunnelRatioScalar", 1.0);
    private static final WaltTunable kSpindexerRatioMultiplier = new WaltTunable("/Indexer/Spindexer/spindexerRatioScalar", 1.0);

    /* LOGGERS */
    private final String kTunnelLogTab = "/Tunnel";
    private final String kSpindexerLogTab = "/Spindexer";

    private final DoubleLogger log_spindexerRPS = WaltLogger.logDouble(kLogTab + kSpindexerLogTab, "spindexerRPS");
    private final DoubleLogger log_tunnelRPS = WaltLogger.logDouble(kLogTab + kTunnelLogTab, "tunnelRPS");

    private final DoubleLogger log_desiredSpindexerRPS = WaltLogger.logDouble(kLogTab + kSpindexerLogTab, "desiredRPS");
    private final DoubleLogger log_desiredTunnelRPS = WaltLogger.logDouble(kLogTab + kTunnelLogTab, "desiredRPS");
    private final DoubleLogger log_spindexerStatorCurrent = WaltLogger.logDouble(kLogTab + kSpindexerLogTab, "statorCurrent");
    private final DoubleLogger log_spindexerSupplyCurrent = WaltLogger.logDouble(kLogTab + kSpindexerLogTab, "supplyCurrent");

    private final StatusSignal<AngularVelocity> sig_spindexerVelo = m_spindexer.getVelocity();
    private final StatusSignal<Current> sig_spindexerStatorCurrent = m_spindexer.getStatorCurrent();
    private final StatusSignal<Current> sig_spindexerSupplyCurrent = m_spindexer.getSupplyCurrent();;
    private final StatusSignal<AngularVelocity> sig_tunnelVelo = m_tunnel.getVelocity();
    private final StatusSignal<Double> sig_tunnelCLErr = m_tunnel.getClosedLoopError();

    private final DoubleLogger log_tunnelClosedLoopError = WaltLogger.logDouble(kLogTab + kTunnelLogTab, "closedLoopError");
    private final BooleanLogger log_isTunnelSpunUp = WaltLogger.logBoolean(kLogTab + kTunnelLogTab, "spunUp");

    private boolean m_isTunnelSpunUp = false;
    private double m_tunnelVelocityRotPerSec = 0.0;
    private double m_desiredTunnelRPS = 0.0;
    private double m_desiredSpindexerRPS = 0.0;

    /* CONSTRUCTOR */
    public Indexer() {
        m_spindexer.getConfigurator().apply(kSpindexerTalonFXConfiguration);
        m_tunnel.getConfigurator().apply(kTunnelTalonFXConfiguration);

        SignalManager.register(Constants.kCanivoreBus, sig_spindexerVelo, sig_tunnelVelo, sig_tunnelCLErr, sig_spindexerStatorCurrent, sig_spindexerSupplyCurrent);

        initSim();
    }

    //TODO: Change orientation if necessary
    private void initSim() {
        WaltMotorSim.initSimFX(m_spindexer, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
        WaltMotorSim.initSimFX(m_tunnel, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
    }

    /* COMMANDS */
    //---STARTS AND STOPS
    public Command startIndexerCmd() {
        return Commands.sequence(
            startTunnelCmd(),
            startSpindexerCmd()
        );
    }

    public Command stopIndexerCmd() {
        return Commands.sequence(
            stopTunnelCmd(),
            stopSpindexerCmd()
        );
    }

    public Command startSpindexerCmd() {
        return setSpindexerVelocityCmd(kSpindexerShootRPS);
    }

    public Command stopSpindexerCmd() {
        return setSpindexerVelocityCmd(RotationsPerSecond.zero());
    }

    public void stopSpindexer() {
        setSpindexerVelocity(0);
    }

    public void setIndexerFromShooterRPS(DoubleSupplier shooterRPS) {
        setTunnelVelocity(tunnelRPSFromShooter(shooterRPS));
        setSpindexerVelocity(spindexerRPSFromShooter(shooterRPS));
    }

    public Command startTunnelCmd() {
        return setTunnelVelocityCmd(kTunnelShootRPS);
    }

    public Command stopTunnelCmd() {
        return setTunnelVelocityCmd(RotationsPerSecond.zero());
    }

    public void stopTunnel() {
        setTunnelVelocity(0);
    }

    private void refreshTunnelState() {
        m_tunnelVelocityRotPerSec = sig_tunnelVelo.getValueAsDouble();
        log_tunnelRPS.accept(m_tunnelVelocityRotPerSec);

        log_tunnelClosedLoopError.accept(sig_tunnelCLErr.getValueAsDouble());
        m_isTunnelSpunUp = sig_tunnelCLErr.isNear(0, 3);
        log_isTunnelSpunUp.accept(m_isTunnelSpunUp);
    }

    public boolean isTunnelSpunUp() {
        return m_isTunnelSpunUp;
    }

    public double getTunnelVelocityRotPerSec() {
        return m_tunnelVelocityRotPerSec;
    }

    public double getDesiredTunnelVelocityRPS() {
        return m_desiredTunnelRPS;
    }

    public double getDesiredSpindexerVelocityRPS() {
        return m_desiredSpindexerRPS;
    }

    //---SPINDEXER
    public void setSpindexerVelocity(double RPS) {
        if (RPS == 0) {
            m_spindexer.setControl(m_spindexerVelocityRequest.withVelocity(0));
            m_spindexer.setControl(m_spindexerMotorIdleReq);
        } else {
            RPS = kSpindexerRPSOverride.enabled() ? kSpindexerRPSOverride.get() : RPS; 
            m_spindexer.setControl(m_spindexerVelocityRequest.withVelocity(RPS));
        }
        m_desiredSpindexerRPS = RPS;
        log_desiredSpindexerRPS.accept(RPS);
    }

    public Command setSpindexerVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setSpindexerVelocity(RPS.in(RotationsPerSecond)));
    }

    //---TUNNEL
    public void setTunnelVelocity(double RPS) {
        if (RPS == 0) {
            m_tunnel.setControl(m_tunnelVelocityRequest.withVelocity(0));
            m_tunnel.setControl(m_tunnelMotorIdleReq);
        } else {
            RPS = kTunnelRPSOverride.enabled() ? kTunnelRPSOverride.get() : RPS; 
            m_tunnel.setControl(m_tunnelVelocityRequest.withVelocity(RPS));
        }
        m_desiredTunnelRPS = RPS;
        log_desiredTunnelRPS.accept(RPS);
    }

    public Command setTunnelVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setTunnelVelocity(RPS.in(RotationsPerSecond)));
    }

    //STATICS for conversions
    public static double tunnelRPSFromShooter(DoubleSupplier shooterRPS) {
        return Math.min(shooterRPS.getAsDouble() * kTunnelFromShooterRatio * (kTunnelRatioMultiplier.enabled() ? kTunnelRatioMultiplier.get() : 1.0) , kTunnelMaxRPSD);
    }

    public static double spindexerRPSFromShooter(DoubleSupplier shooterRPS) {
        return Math.min(shooterRPS.getAsDouble() * kSpindexerFromShooterRatio * (kSpindexerRatioMultiplier.enabled() ? kSpindexerRatioMultiplier.get() : 1.0), kSpindexerMaxRPSD);
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        log_spindexerRPS.accept(sig_spindexerVelo.getValueAsDouble());
        log_spindexerStatorCurrent.accept(sig_spindexerStatorCurrent.getValueAsDouble());
        log_spindexerSupplyCurrent.accept(sig_spindexerSupplyCurrent.getValueAsDouble());
        refreshTunnelState();
    }

    // @Override
    // public void simulationPeriodic() {
    //     WaltMotorSim.updateSimFX(m_tunnel, m_tunnelSim);
    //     WaltMotorSim.updateSimFX(m_spindexer, m_spindexerSim);
    // }
}
