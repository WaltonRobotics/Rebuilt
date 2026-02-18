package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.IndexerK.*;

import frc.robot.Constants;
import frc.util.MotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Indexer extends SubsystemBase {
    /* VARIABLES */
    // Motors and Control Requests
    private final TalonFX m_spindexer = new TalonFX(kSpindexerCANID); // X60
    private final TalonFX m_tunnel = new TalonFX(kTunnelCANID); // X60

    private final VelocityVoltage m_spindexerVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage m_tunnelVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final AngularVelocity m_spindexerRPS = RotationsPerSecond.of(30);
    private final AngularVelocity m_tunnelRPS = RotationsPerSecond.of(108);

    // Simulation
    private final DCMotorSim m_spindexerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            kSpindexerMOI,
            kSpindexerGearing
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final DCMotorSim m_tunnelSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            kTunnelMOI,
            kTunnelGearing
        ),
        DCMotor.getKrakenX60Foc(1)
    );
    
    // Loggers
    private final DoubleLogger log_spindexerRPS = WaltLogger.logDouble(kLogTab, "spindexerRPS");
    private final DoubleLogger log_tunnelRPS = WaltLogger.logDouble(kLogTab, "tunnelRPS");

    /* CONSTRUCTOR */
    public Indexer() {
        m_spindexer.getConfigurator().apply(kSpindexerTalonFXConfiguration);
        m_tunnel.getConfigurator().apply(kTunnelTalonFXConfiguration);

        initSim();
    }

    //TODO: Change orientation if necessary
    private void initSim() {
        MotorSim.initSimFX(m_spindexer, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
        MotorSim.initSimFX(m_tunnel, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
    }

    /* COMMANDS */
    public Command startSpindexer() {
        return setSpindexerVelocityCmd(m_spindexerRPS);
    }

    public Command stopSpindexer() {
        return setSpindexerVelocityCmd(RotationsPerSecond.of(0));
    }

    public Command startTunnel() {
        return setTunnelVelocityCmd(m_tunnelRPS);
    }

    public Command stopTunnel() {
        return setTunnelVelocityCmd(RotationsPerSecond.of(0));
    }

    public Command setSpindexerVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_spindexer.setControl(m_spindexerVelocityRequest.withVelocity(RPS)));
    }

    public Command setTunnelVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_tunnel.setControl(m_tunnelVelocityRequest.withVelocity(RPS)));
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        log_spindexerRPS.accept(m_spindexer.getVelocity().getValueAsDouble());
        log_tunnelRPS.accept(m_tunnel.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        MotorSim.updateSimFX(m_tunnel, m_tunnelSim);
        MotorSim.updateSimFX(m_spindexer, m_spindexerSim);
    }
}
