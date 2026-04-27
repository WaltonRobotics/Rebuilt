package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IndexerK.*;

import frc.robot.Constants;
import frc.util.SignalManager;
import frc.util.WaltMotorSim;
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
    
    /* LOGGERS */
    private final DoubleLogger log_spindexerRPS = WaltLogger.logDouble(kLogTab, "spindexerRPS");
    private final DoubleLogger log_tunnelRPS = WaltLogger.logDouble(kLogTab, "tunnelRPS");

    private final DoubleLogger log_desiredSpindexerRPS = WaltLogger.logDouble(kLogTab, "desiredSpindexerRPS");
    private final DoubleLogger log_desiredTunnelRPS = WaltLogger.logDouble(kLogTab, "desiredTunnelRPS");
    private final DoubleLogger log_spindexerStatorCurrent = WaltLogger.logDouble(kLogTab, "spindexerStatorCurrent");
    private final DoubleLogger log_spindexerSupplyCurrent = WaltLogger.logDouble(kLogTab, "spindexerSupplyCurrent");

    private final StatusSignal<AngularVelocity> sig_spindexerVelo = m_spindexer.getVelocity();
    private final StatusSignal<Current> sig_spindexerStatorCurrent = m_spindexer.getStatorCurrent();
    private final StatusSignal<Current> sig_spindexerSupplyCurrent = m_spindexer.getSupplyCurrent();;
    private final StatusSignal<AngularVelocity> sig_tunnelVelo = m_tunnel.getVelocity();
    private final StatusSignal<Double> sig_tunnelCLErr = m_tunnel.getClosedLoopError();

    private final DoubleLogger log_tunnelClosedLoopError = WaltLogger.logDouble(kLogTab, "tunnelClosedLoopError");
    private final BooleanLogger log_isTunnelSpunUp = WaltLogger.logBoolean(kLogTab, "isTunnelSpunUp");


    private boolean m_isTunnelSpunUp = false;
    private double m_tunnelVelocityRotPerSec = 0.0;

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
        setSpindexerVelocity(RotationsPerSecond.zero());
    }

    public Command startTunnelCmd() {
        return setTunnelVelocityCmd(kTunnelShootRPS);
    }

    public Command stopTunnelCmd() {
        return setTunnelVelocityCmd(RotationsPerSecond.zero());
    }

    public void stopTunnel() {
        setTunnelVelocity(RotationsPerSecond.zero());
    }

    private void refreshTunnelState() {
        m_tunnelVelocityRotPerSec = sig_tunnelVelo.getValueAsDouble();
        log_tunnelRPS.accept(m_tunnelVelocityRotPerSec);

        log_tunnelClosedLoopError.accept(sig_tunnelCLErr.getValueAsDouble());
        m_isTunnelSpunUp = sig_tunnelCLErr.isNear(0, 30);
        log_isTunnelSpunUp.accept(m_isTunnelSpunUp);
    }

    public boolean isTunnelSpunUp() {
        return m_isTunnelSpunUp;
    }

    public double getTunnelVelocityRotPerSec() {
        return m_tunnelVelocityRotPerSec;
    }

    //---SPINDEXER
    public void setSpindexerVelocity(AngularVelocity RPS) {
        m_spindexer.setControl(m_spindexerVelocityRequest.withVelocity(RPS));
        log_desiredSpindexerRPS.accept(RPS.in(RotationsPerSecond));
    }

    public Command setSpindexerVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setSpindexerVelocity(RPS));
    }

    //---TUNNEL
    public void setTunnelVelocity(AngularVelocity RPS) {
        m_tunnel.setControl(m_tunnelVelocityRequest.withVelocity(RPS));
        log_desiredTunnelRPS.accept(RPS.in(RotationsPerSecond));
    }

    public Command setTunnelVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setTunnelVelocity(RPS));
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
