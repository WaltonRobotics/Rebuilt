package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.ChassisReference;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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

    private MotionMagicVoltage m_MMVReq = new MotionMagicVoltage(0).withEnableFOC(true);
    private VelocityVoltage m_VVReq = new VelocityVoltage(0).withEnableFOC(true);

    // Loggers
    DoubleLogger log_deployRots = WaltLogger.logDouble(IntakeK.kLogTab, "deployRots");
    DoubleLogger log_targetDeployRots = WaltLogger.logDouble(IntakeK.kLogTab, "targetDeployRots");
    DoubleLogger log_rollerRPS = WaltLogger.logDouble(IntakeK.kLogTab, "rollerRPS");
    DoubleLogger log_targetRollerRPS = WaltLogger.logDouble(IntakeK.kLogTab, "targetRollerRPS");

    // Simulators
    private final DCMotorSim m_deploySim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            IntakeK.kDeployMomentOfInertia,
            IntakeK.kDeployGearing
        ),
        DCMotor.getKrakenX60Foc(1)
    );

    private final DCMotorSim m_rollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1),
            IntakeK.kRollerMomentOfInertia,
            IntakeK.kRollerGearing
        ),
        DCMotor.getKrakenX44Foc(1) // returns gearbox
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

    public Command setDeployPos(DeployPosition rots) {
        return setDeployPos(rots.rots);
    }

    public Command setDeployPos(Angle rots) {
        return runOnce(() -> m_deployMotor.setControl(m_MMVReq.withPosition(rots)));
    }

    public Command setRollerSpeed(RollerVelocity RPS) {
        return setRollerSpeed(RPS.RPS);
    }

    public Command setRollerSpeed(AngularVelocity RPS) {
        return runOnce(() -> m_rollerMotor.setControl(m_VVReq.withVelocity(RPS)));
    }

    @Override
    public void periodic() {
        log_targetDeployRots.accept(m_MMVReq.Position);
        log_targetRollerRPS.accept(m_VVReq.Velocity);
        log_rollerRPS.accept(m_rollerMotor.getVelocity().getValueAsDouble());
        log_deployRots.accept(m_deployMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var deployFXSim = m_deployMotor.getSimState();

        m_deploySim.setInputVoltage(deployFXSim.getMotorVoltage());
        m_deploySim.update(0.02);
        
        deployFXSim.setRawRotorPosition(m_deploySim.getAngularPositionRotations() * IntakeK.kDeployGearing);
        deployFXSim.setRotorVelocity(m_deploySim.getAngularVelocity().times(IntakeK.kDeployGearing));
        deployFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var rollerFXSim = m_rollerMotor.getSimState();

        m_rollerSim.setInputVoltage(rollerFXSim.getMotorVoltage());
        m_rollerSim.update(0.02);
        
        rollerFXSim.setRawRotorPosition(m_rollerSim.getAngularPositionRotations() * IntakeK.kRollerGearing);
        rollerFXSim.setRotorVelocity(m_rollerSim.getAngularVelocity().times(IntakeK.kRollerGearing));
        rollerFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    public enum DeployPosition{
        RETRACTED(0),
        SAFE(72),
        DEPLOYED(87.485);

        private Angle degs;
        private Angle rots;

        private DeployPosition(double degs) {
            this.degs = Degrees.of(degs);
            this.rots = Rotations.of(Rotations.convertFrom(this.degs.magnitude(), Degrees));
        }
    }

    public enum RollerVelocity{
        MAX(50),
        MID(33),
        STOP(0);

        private AngularVelocity RPS;

        private RollerVelocity(double RPS) {
            this.RPS = RotationsPerSecond.of(RPS);
        }
    }
}