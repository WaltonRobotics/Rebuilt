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
    private final TalonFX m_spinner = new TalonFX(kSpinnerCANID); // X60Foc
    private final TalonFX m_exhaust = new TalonFX(kExhaustCANID); // X60Foc

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
    private final DoubleLogger log_spinnerRPS = WaltLogger.logDouble(kLogTab, "spinnerRPS");
    private final DoubleLogger log_exhaustRPS = WaltLogger.logDouble(kLogTab, "exhaustRPS");

    /* CONSTRUCTOR */
    public Indexer() {
        m_spinner.getConfigurator().apply(kSpinnerTalonFXConfiguration);
        m_exhaust.getConfigurator().apply(kExhaustTalonFXConfiguration);

        initSim();
    }

    //TODO: Change orientation if necessary
    private void initSim() {
        MotorSim.initSimFX(m_spinner, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
        MotorSim.initSimFX(m_exhaust, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
    }

    /* COMMANDS */
    public Command startSpinner() {
        return setSpinnerVelocityCmd(m_spinnerRPS);
    }

    public Command stopSpinner() {
        return setSpinnerVelocityCmd(RotationsPerSecond.of(0));
    }

    public Command startExhaust() {
        return setExhaustVelocityCmd(m_exhaustRPS);
    }

    public Command stopExhaust() {
        return setExhaustVelocityCmd(RotationsPerSecond.of(0));
    }

    public Command setSpinnerVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_spinner.setControl(m_spinnerVelocityRequest.withVelocity(RPS)));
    }

    public Command setExhaustVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_exhaust.setControl(m_exhaustVelocityRequest.withVelocity(RPS)));
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        log_spinnerRPS.accept(m_spinner.getVelocity().getValueAsDouble());
        log_exhaustRPS.accept(m_exhaust.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        MotorSim.updateSimFX(m_exhaust, m_exhaustSim);
        MotorSim.updateSimFX(m_spinner, m_spinnerSim);
    }
}
