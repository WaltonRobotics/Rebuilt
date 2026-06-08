package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

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

/*
 * Indexer
 *
 * The indexer sits between the intake and the shooter and is responsible for
 * moving balls from the hopper into the shooter at the right speed.
 *
 * It has two motors:
 *   Spindexer — a spinning disk/floor underneath the ball hopper. It spins
 *               to rotate balls around and push them toward the tunnel entrance.
 *   Tunnel    — a belt or tube that carries balls linearly from the hopper
 *               toward the shooter flywheel. It needs to spin fast enough to
 *               match the ball's exit speed.
 *
 * The tunnel speed is linked to the shooter flywheel speed via a geometric ratio
 * derived from the physical radii of each wheel (see tunnelRPSFromShooter).
 * This prevents the ball from being jerked or slowed as it enters the shooter.
 */
public class Indexer extends SubsystemBase {

    // ---- motors ----

    // Both motors are Kraken X60s on the CANivore bus.
    // X60Foc = Kraken X60 with Field-Oriented Control enabled (more torque at high speeds).
    private final TalonFX m_spindexer = new TalonFX(kSpindexerCANID, Constants.kCanivoreBus);
    private final TalonFX m_tunnel = new TalonFX(kTunnelCANID, Constants.kCanivoreBus);

    // ---- control requests ----
    // VelocityVoltage tells the motor controller to hold a specific speed (in RPS)
    // using a PID loop + feedforward. We pre-create these and mutate them with
    // .withVelocity() to avoid unnecessary object allocation on the hot path.

    // Spindexer uses non-FOC velocity (simpler, lower torque demands).
    private final VelocityVoltage m_spindexerVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    // Tunnel uses FOC velocity for smoother, more precise speed matching with the shooter.
    private final VelocityVoltage m_tunnelVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    // CoastOut lets the motor spin freely to a stop instead of actively braking.
    // We use this when stopping so the ball's momentum doesn't fight the motor.
    private final CoastOut m_spindexerMotorIdleReq = new CoastOut();
    private final CoastOut m_tunnelMotorIdleReq = new CoastOut();

    // ---- tunables ----
    // WaltTunable values can be adjusted at runtime through NetworkTables (no redeploy needed).
    // If a tunable has been set, getOr() returns the tuned value; otherwise it returns the default.
    // Useful during practice to dial in speeds without redeploying.

    private static final WaltTunable kTunnelRPSOverride = new WaltTunable("/Indexer/Tunnel/tunnelRPSOverride",         kTunnelShootRPSD);
    private static final WaltTunable kSpindexerRPSOverride = new WaltTunable("/Indexer/Spindexer/spindexerRPSOverride",   kSpindexerShootRPSD);
    private static final WaltTunable kTunnelRatioScalarTuner = new WaltTunable("/Indexer/Tunnel/tunnelRatioScalar",         1.0);
    private static final WaltTunable kSpindexerRatioScalarTuner = new WaltTunable("/Indexer/Spindexer/spindexerRatioScalar",   1.0);

    // ---- loggers ----

    private final String kTunnelLogTab = "/Tunnel";
    private final String kSpindexerLogTab = "/Spindexer";

    private final DoubleLogger log_spindexerRPS = WaltLogger.logDouble(kLogTab + kSpindexerLogTab, "spindexerRPS");
    private final DoubleLogger log_tunnelRPS = WaltLogger.logDouble(kLogTab + kTunnelLogTab,    "tunnelRPS");
    private final DoubleLogger log_desiredSpindexerRPS = WaltLogger.logDouble(kLogTab + kSpindexerLogTab, "desiredRPS");
    private final DoubleLogger log_desiredTunnelRPS = WaltLogger.logDouble(kLogTab + kTunnelLogTab,    "desiredRPS");
    private final DoubleLogger log_spindexerStatorCurrent = WaltLogger.logDouble(kLogTab + kSpindexerLogTab, "statorCurrent");
    private final DoubleLogger log_spindexerSupplyCurrent = WaltLogger.logDouble(kLogTab + kSpindexerLogTab, "supplyCurrent");
    private final DoubleLogger log_tunnelClosedLoopError = WaltLogger.logDouble(kLogTab + kTunnelLogTab,    "closedLoopError");
    private final BooleanLogger log_isTunnelSpunUp = WaltLogger.logBoolean(kLogTab + kTunnelLogTab,   "spunUp");

    // ---- status signals ----
    // StatusSignals are CTRE's way of efficiently reading motor data without spamming the CAN bus.
    // We register them with SignalManager so they get refreshed at a consistent rate each loop.

    private final StatusSignal<AngularVelocity> sig_spindexerVelo = m_spindexer.getVelocity();
    private final StatusSignal<Current> sig_spindexerStatorCurrent = m_spindexer.getStatorCurrent();
    private final StatusSignal<Current> sig_spindexerSupplyCurrent = m_spindexer.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> sig_tunnelVelo = m_tunnel.getVelocity();
    private final StatusSignal<Double> sig_tunnelCLErr = m_tunnel.getClosedLoopError();

    // ---- state ----

    private boolean m_isTunnelSpunUp = false;
    private double m_tunnelVelocityRotPerSec = 0.0;
    private double m_desiredTunnelRPS = 0.0;
    private double m_desiredSpindexerRPS = 0.0;


    // =============================================================
    // CONSTRUCTOR
    // =============================================================

    public Indexer() {
        // Apply the motor configurations defined in Constants.IndexerK.
        m_spindexer.getConfigurator().apply(kSpindexerTalonFXConfiguration);
        m_tunnel.getConfigurator().apply(kTunnelTalonFXConfiguration);

        // Register all status signals so they're refreshed every loop at the correct rate.
        SignalManager.register(Constants.kCanivoreBus,
            sig_spindexerVelo, sig_tunnelVelo, sig_tunnelCLErr,
            sig_spindexerStatorCurrent, sig_spindexerSupplyCurrent);

        initSim();
    }

    // Sets up the simulation models for both motors.
    // TODO: Change orientation if mechanical setup changes.
    private void initSim() {
        WaltMotorSim.initSimFX(m_spindexer, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
        WaltMotorSim.initSimFX(m_tunnel, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
    }


    // =============================================================
    // COMMANDS — START / STOP
    // =============================================================

    // Starts both the tunnel and spindexer at their shoot speeds.
    public Command startIndexerCmd() {
        return Commands.sequence(startTunnelCmd(), startSpindexerCmd());
    }

    // Stops both motors by commanding 0 RPS (which triggers CoastOut in the setters).
    public Command stopIndexerCmd() {
        return Commands.sequence(stopTunnelCmd(), stopSpindexerCmd());
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

    public Command startTunnelCmd() {
        return setTunnelVelocityCmd(kTunnelShootRPS);
    }

    public Command stopTunnelCmd() {
        return setTunnelVelocityCmd(RotationsPerSecond.zero());
    }

    public void stopTunnel() {
        setTunnelVelocity(0);
    }

    // Sets both indexer motors to speeds derived from the shooter's current RPS.
    // This keeps the indexer surface speed matched to the flywheel so the ball
    // doesn't get jerked when it enters the shooter.
    public void setIndexerFromShooterRPS(DoubleSupplier shooterRPS) {
        setTunnelVelocity(tunnelRPSFromShooter(shooterRPS).getAsDouble());
        setSpindexerVelocity(spindexerRPSFromShooter(shooterRPS).getAsDouble());
    }


    // =============================================================
    // SPINDEXER CONTROL
    // =============================================================

    // When stopping (RPS == 0) we switch to CoastOut so the spindexer spins
    // down freely
    public void setSpindexerVelocity(double RPS) {
        if (RPS == 0) {
            m_spindexer.setControl(m_spindexerVelocityRequest.withVelocity(0));
            m_spindexer.setControl(m_spindexerMotorIdleReq);
        } else {
            RPS = kSpindexerRPSOverride.getOr(RPS); // override if tuned at runtime
            m_spindexer.setControl(m_spindexerVelocityRequest.withVelocity(RPS));
        }
        m_desiredSpindexerRPS = RPS;
        log_desiredSpindexerRPS.accept(RPS);
    }

    public Command setSpindexerVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setSpindexerVelocity(RPS.in(RotationsPerSecond)));
    }


    // =============================================================
    // TUNNEL CONTROL
    // =============================================================

    // Same coast-on-stop pattern as the spindexer.
    public void setTunnelVelocity(double RPS) {
        if (RPS == 0) {
            m_tunnel.setControl(m_tunnelVelocityRequest.withVelocity(0));
            m_tunnel.setControl(m_tunnelMotorIdleReq);
        } else {
            RPS = kTunnelRPSOverride.getOr(RPS); // override if tuned at runtime
            m_tunnel.setControl(m_tunnelVelocityRequest.withVelocity(RPS));
        }
        m_desiredTunnelRPS = RPS;
        log_desiredTunnelRPS.accept(RPS);
    }

    public Command setTunnelVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> setTunnelVelocity(RPS.in(RotationsPerSecond)));
    }

    // Polls the tunnel's speed and closed-loop error, then decides if it's "spun up."
    // The closed-loop error is how far off the tunnel is from its target speed (in RPS).
    // At very high speeds (> 80 RPS), we allow a larger error tolerance since the
    // motor is working harder and small deviations matter less.
    private void refreshTunnelState() {
        m_tunnelVelocityRotPerSec = sig_tunnelVelo.getValueAsDouble();
        log_tunnelRPS.accept(m_tunnelVelocityRotPerSec);
        log_tunnelClosedLoopError.accept(sig_tunnelCLErr.getValueAsDouble());

        m_isTunnelSpunUp = sig_tunnelCLErr.isNear(0, 3); // within 3 RPS of target
        if (m_desiredTunnelRPS > 80) {
            m_isTunnelSpunUp = sig_tunnelCLErr.isNear(0, 6); // looser tolerance at high speed
        }

        log_isTunnelSpunUp.accept(m_isTunnelSpunUp);
    }

    public boolean isTunnelSpunUp() { return m_isTunnelSpunUp; }
    public double getTunnelVelocityRotPerSec() { return m_tunnelVelocityRotPerSec; }
    public double getDesiredTunnelVelocityRPS() { return m_desiredTunnelRPS; }
    public double getDesiredSpindexerVelocityRPS() { return m_desiredSpindexerRPS; }


    // =============================================================
    // SPEED RATIO CONVERSIONS
    // =============================================================

    // The tunnel and spindexer speeds are derived from the shooter flywheel speed
    // using the physical radii of each wheel/pulley. The goal is to match surface
    // speeds so the ball doesn't get grabbed or slowed when it transitions between
    // the indexer and the shooter.
    //
    // Surface speed = radius × angular velocity
    // For the ball to travel smoothly: r_shooter × ω_shooter = r_indexer × ω_indexer
    // → ω_indexer = (r_shooter / r_indexer) × ω_shooter
    //
    // The ratio constants (kTunnelFromShooterRatio, kSpindexerFromShooterRatio) are
    // pre-computed in Constants.IndexerK from the physical radii.
    // kTunnelRatioScalarTuner allows fine-tuning that ratio at runtime.

    public static DoubleSupplier tunnelRPSFromShooter(DoubleSupplier shooterRPS) {
        return () -> Math.min(
            shooterRPS.getAsDouble() * kTunnelFromShooterRatio * kTunnelRatioScalarTuner.getOr(1.0),
            kTunnelMaxRPSD
        );
    }

    public static DoubleSupplier spindexerRPSFromShooter(DoubleSupplier shooterRPS) {
        return () -> Math.min(
            shooterRPS.getAsDouble() * kSpindexerFromShooterRatio * kSpindexerRatioScalarTuner.getOr(1.0),
            kSpindexerMaxRPSD
        );
    }


    // =============================================================
    // PERIODIC
    // =============================================================

    @Override
    public void periodic() {
        log_spindexerRPS.accept(sig_spindexerVelo.getValueAsDouble());
        log_spindexerStatorCurrent.accept(sig_spindexerStatorCurrent.getValueAsDouble());
        log_spindexerSupplyCurrent.accept(sig_spindexerSupplyCurrent.getValueAsDouble());
        refreshTunnelState();
    }
}
