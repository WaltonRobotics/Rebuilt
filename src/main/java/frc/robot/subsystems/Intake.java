package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.WaltLogger.DoubleLogger;

import frc.robot.Constants.IntakeK;
import frc.util.WaltLogger;

public class Intake extends SubsystemBase {
    private final TalonFX m_deployMotor = new TalonFX(IntakeK.kDeployCANID);
    private final TalonFX m_rollerMotor = new TalonFX(IntakeK.kRollerCANID);

    private MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0);
    private VelocityVoltage m_veloVoltage = new VelocityVoltage(0);

    // Loggers
    DoubleLogger log_targetDeployPos = WaltLogger.logDouble(IntakeK.kLogTab, "targetDeployPos");
    DoubleLogger log_rollerVelocity = WaltLogger.logDouble(IntakeK.kLogTab, "rollerVelo");
    DoubleLogger log_deployPosition = WaltLogger.logDouble(IntakeK.kLogTab, "deployPos");
    DoubleLogger log_targetRollerVelo = WaltLogger.logDouble(IntakeK.kLogTab, "targetRollerVelo");

    // Simulators
    private final DCMotorSim m_deploySim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1),
            IntakeK.kDeployMomentOfInertia,
            IntakeK.kDeployGearing
        ),
        DCMotor.getKrakenX44(1)
    );

    private final DCMotorSim m_rollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1),
            IntakeK.kRollerMomentOfInertia,
            IntakeK.kRollerGearing
        ),
        DCMotor.getKrakenX44(1) // returns gearbox
    );

    public Intake() {
        m_deployMotor.getConfigurator().apply(IntakeK.kDeployConfiguration);
        m_rollerMotor.getConfigurator().apply(IntakeK.kRollerConfiguration);
        initSim();
    }

    private void initSim() {
        var deployFXSim = m_deployMotor.getSimState();
        deployFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        deployFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        var rollerFXSim = m_deployMotor.getSimState();
        rollerFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        rollerFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    public Command spinRollers(double rps) {
        return runOnce(() -> m_rollerMotor.setControl(m_veloVoltage.withVelocity(rps)));
    }

    public Command stopRollers() {
        return runOnce(() -> m_rollerMotor.setControl(m_veloVoltage.withVelocity(0)));
    }

    public Command deployToRots(double rots) {
        return runOnce(() -> m_deployMotor.setControl(m_MMVRequest.withPosition(rots)));
    }

    @Override
    public void periodic() {
        log_targetDeployPos.accept(m_MMVRequest.Position);
        log_targetRollerVelo.accept(m_veloVoltage.Velocity);
        log_rollerVelocity.accept(m_rollerMotor.getVelocity().getValueAsDouble());
        log_deployPosition.accept(m_deployMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var deployFXSim = m_deployMotor.getSimState();

        m_deploySim.setInputVoltage(deployFXSim.getMotorVoltage());
        m_deploySim.update(0.02);
        
        deployFXSim.setRawRotorPosition(m_deploySim.getAngularPositionRotations());
        deployFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var rollerFXSim = m_rollerMotor.getSimState();

        m_rollerSim.setInputVoltage(rollerFXSim.getMotorVoltage());
        m_rollerSim.update(0.02);
        
        rollerFXSim.setRotorVelocity(m_rollerSim.getAngularVelocityRPM() / 60);

        rollerFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}