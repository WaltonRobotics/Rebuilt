package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.TransferK.*;

import frc.robot.Constants;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Transfer extends SubsystemBase {
    /* VARIABLES */
    // Motors
    private final TalonFX m_spinner = new TalonFX(kSpinnerCANID); // X60
    private final TalonFX m_exhaust = new TalonFX(kExhaustCANID); // X60

    // Simulation, TODO: update sim values (J and gearing)
    private final DCMotorSim m_spinnerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.01,                   
            1.5
        ),
        DCMotor.getKrakenX60(1)
    );

    private final DCMotorSim m_exhaustSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.01,                    
            1.5
        ),
        DCMotor.getKrakenX60(1)
    );  
    
    // Loggers
    private final DoubleLogger log_spinnerVelocityRPS = WaltLogger.logDouble(kLogTab, "shooterVelocityRPS");
    private final DoubleLogger log_exhaustVelocityRPS = WaltLogger.logDouble(kLogTab, "exhaustVelocityRPS");

    /* CONSTRUCTOR */
    public Transfer() {
        m_spinner.getConfigurator().apply(kSpinnerSlot0Configs);
        m_exhaust.getConfigurator().apply(kExhaustSlot1Configs);

        initSim();
    }

    public void startSpinner(double rps) {
        m_spinner.setControl(new VelocityVoltage(rps));
    }

    public void stopSpinner() {
        m_spinner.setControl(new VelocityVoltage(0));
    }

    public void startExhaust(double rps) {
        m_exhaust.setControl(new VelocityVoltage(rps));
    }

    public void stopExhaust() {
        m_exhaust.setControl(new VelocityVoltage(0));
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

    /* PERIODICS */
    @Override
    public void periodic() {
        log_spinnerVelocityRPS.accept(m_spinner.getVelocity().getValueAsDouble());
        log_exhaustVelocityRPS.accept(m_exhaust.getVelocity().getValueAsDouble());

        initSim();
    }

    @Override
    public void simulationPeriodic() {
        // Spinner  
        var m_spinnerFXSim = m_spinner.getSimState();

        m_spinnerSim.setInputVoltage(m_spinnerFXSim.getMotorVoltage());
        m_spinnerSim.update(Constants.kSimPeriodicUpdateInterval);

        m_spinnerFXSim.setRawRotorPosition(m_spinnerSim.getAngularPositionRotations());
        m_spinnerFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Exhaust
        var m_exhaustFXSim = m_exhaust.getSimState();
        
        m_exhaustSim.setInputVoltage(m_exhaustFXSim.getMotorVoltage());
        m_exhaustSim.update(Constants.kSimPeriodicUpdateInterval);

        m_exhaustFXSim.setRawRotorPosition(m_exhaustSim.getAngularPositionRotations());
        m_exhaustFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
