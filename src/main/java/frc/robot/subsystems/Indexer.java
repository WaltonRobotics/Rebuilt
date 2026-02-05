package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

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

    private final VelocityVoltage m_spinnerVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage m_exhaustVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    // Simulation
    private final DCMotorSim m_spinnerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            kSpinnerMOI,
            kSpinnerGearing
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final DCMotorSim m_exhaustSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            kExhaustMOI,
            kExhaustGearing
        ),
        DCMotor.getKrakenX60Foc(1)
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
        initTalonFXSimState(m_spinner,
        ChassisReference.CounterClockwise_Positive,
        MotorType.KrakenX60);
        
        initTalonFXSimState(m_exhaust,
        ChassisReference.CounterClockwise_Positive,
        MotorType.KrakenX60);
    }

    private void initTalonFXSimState(TalonFX motor, ChassisReference orientation, MotorType motorType) {
        motor.getSimState().Orientation = orientation;
        motor.getSimState().setMotorType(motorType);
    }
    
    public Command updateSpinner(double RPS) {
        return runOnce(() -> m_spinner.setControl(m_spinnerVelocityRequest.withVelocity(RPS)));
    }

    public Command updateExhaust(double RPS) {
        return runOnce(() -> m_exhaust.setControl(m_exhaustVelocityRequest.withVelocity(RPS)));
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
        var spinnerFXSim = m_spinner.getSimState();

        m_spinnerSim.setInputVoltage(spinnerFXSim.getMotorVoltage());
        m_spinnerSim.update(Constants.kSimPeriodicUpdateInterval);

        spinnerFXSim.setRotorVelocity(m_spinnerSim.getAngularVelocity().times(kSpinnerGearing));
        spinnerFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Exhaust
        var exhaustFXSim = m_exhaust.getSimState();
        
        m_exhaustSim.setInputVoltage(exhaustFXSim.getMotorVoltage());
        m_exhaustSim.update(Constants.kSimPeriodicUpdateInterval);

        exhaustFXSim.setRotorVelocity(m_exhaustSim.getAngularVelocity().times(kExhaustGearing));
        exhaustFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
