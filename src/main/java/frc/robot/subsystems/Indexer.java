package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IndexerK.*;

import frc.robot.Constants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Indexer extends SubsystemBase {
    /* VARIABLES */
    // Motors and Control Requests
    private final TalonFX m_spinner = new TalonFX(kSpinnerCANID); // X60
    private final TalonFX m_exhaust = new TalonFX(kExhaustCANID); // X60

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    // Simulation
    private final DCMotorSim m_spinnerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            kSpinnerMomentOfInertia,
            kSpinnerGearing
        ),
        DCMotor.getKrakenX60(1)
    );

    private final DCMotorSim m_exhaustSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            kExhaustMomentOfInertia,
            kExhaustGearing
        ),
        DCMotor.getKrakenX60(1)
    );  
    
    // Loggers
    private final DoubleLogger log_spinnerVelocityRPS = WaltLogger.logDouble(kLogTab, "spinnerVelocityRPS");
    private final DoubleLogger log_exhaustVelocityRPS = WaltLogger.logDouble(kLogTab, "exhaustVelocityRPS");

    /* CONSTRUCTOR */
    public Indexer() {
        m_spinner.getConfigurator().apply(kSpinnerTalonFXConfiguration);
        m_exhaust.getConfigurator().apply(kExhaustTalonFXConfiguration);

        initSim();
    }

    //TODO: Change orientation if necessary
    private void initSim() {
        var m_spinnerFXSim = m_spinner.getSimState();
        m_spinnerFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        m_spinnerFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        
        var m_exhaustFXSim = m_exhaust.getSimState();
        m_exhaustFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        m_exhaustFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public Command startSpinner(double RPS) {
        return runOnce(() -> m_spinner.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    public Command stopSpinner() {
        return runOnce(() -> m_spinner.setControl(m_velocityRequest.withVelocity(0)));
    }

    public Command startExhaust(double RPS) {
        return runOnce(() -> m_exhaust.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    public Command stopExhaust() {
        return runOnce(() -> m_exhaust.setControl(m_velocityRequest.withVelocity(0)));
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        log_spinnerVelocityRPS.accept(m_spinner.getVelocity().getValueAsDouble());
        log_exhaustVelocityRPS.accept(m_exhaust.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // Spinner  
        var m_spinnerFXSim = m_spinner.getSimState();

        m_spinnerSim.setInputVoltage(m_spinnerFXSim.getMotorVoltage());
        m_spinnerSim.update(Constants.kSimPeriodicUpdateInterval);

        m_spinnerFXSim.setRotorVelocity(m_spinnerSim.getAngularVelocity());
        m_spinnerFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Exhaust
        var m_exhaustFXSim = m_exhaust.getSimState();
        
        m_exhaustSim.setInputVoltage(m_exhaustFXSim.getMotorVoltage());
        m_exhaustSim.update(Constants.kSimPeriodicUpdateInterval);

        m_exhaustFXSim.setRotorVelocity(m_exhaustSim.getAngularVelocity());
        m_exhaustFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
