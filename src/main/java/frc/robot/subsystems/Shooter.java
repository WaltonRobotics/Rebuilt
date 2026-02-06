package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ShooterK.*;

import frc.robot.Constants;
import frc.robot.Constants.ShooterK;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    // motors + control requests
    private final TalonFX m_flywheelLeader = new TalonFX(kLeaderCANID); //X60
    private final TalonFX m_flywheelFollower = new TalonFX(kFollowerCANID); //X60
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final TalonFX m_hood = new TalonFX(kHoodCANID); //X44
    private final PositionVoltage m_positionRequest = new PositionVoltage(0).withEnableFOC(true);

    private final TalonFX m_turret = new TalonFX(kTurretCANID); //X44
    private final MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    // logic booleans
    private boolean m_spunUp = false;

    // beam breaks (if we have one on the shooter)
    public DigitalInput m_exitBeamBreak = new DigitalInput(kExitBeamBreakChannel);

    public final Trigger trg_exitBeamBreak = new Trigger(() -> !m_exitBeamBreak.get()); //true when beam is broken

    public final Trigger exitBeamBreakTrigger(EventLoop loop) {
        return new Trigger(loop, () -> !m_exitBeamBreak.get());
    }

    // sim (TODO: double check constants)
    private final FlywheelSim m_flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(2), 
            kShooterMoI,
            kShooterGearing
        ),
        DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    private final SingleJointedArmSim m_hoodSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX44Foc(1),
            kHoodMoI,
            kHoodGearing
        ),
        DCMotor.getKrakenX44Foc(1),
        kHoodGearing,
        kHoodLength,
        kHoodMinRots.magnitude() * (2*Math.PI),
        kHoodMaxRots.magnitude() * (2*Math.PI),
        false,
        kHoodMinRots.magnitude() * (2*Math.PI)
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1),
            kTurretMoI,
            kTurretGearing
        ),
        DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    // loggers
    private final DoubleLogger log_flywheelVelocityRPS = WaltLogger.logDouble(kLogTab, "flywheelVelocityRPS");
    private final DoubleLogger log_hoodPositionRots = WaltLogger.logDouble(kLogTab, "hoodPositionRots");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab, "turretPositionRots");

    private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    /* CONSTRUCTOR */
    public Shooter() {
        m_flywheelLeader.getConfigurator().apply(kFlywheelLeaderTalonFXConfiguration);
        m_flywheelFollower.getConfigurator().apply(kFlywheelFollowerTalonFXConfiguration);
        m_hood.getConfigurator().apply(kHoodTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);

        m_flywheelFollower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned

        initSim();
    }

    //TODO: update orientation values (if needed)
    private void initSim() {
        var leaderFXSim = m_flywheelLeader.getSimState();
        leaderFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        leaderFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        var hoodFXSim = m_hood.getSimState();
        hoodFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        hoodFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        var turretFXSim = m_turret.getSimState();
        turretFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        turretFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }

    /* COMMANDS */
    // Shooter Commands (Veloity Control)
    public Command setFlywheelVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_flywheelLeader.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    // Hood Commands (Basic Position Control)
    public Command setHoodPositionCmd(Angle rots) {
        return runOnce(() -> m_hood.setControl(m_positionRequest.withPosition(rots)));
    }

    // Turret Commands (Motionmagic Angle Control)
    public Command setTurretPositionCmd(Angle rots) {
        return runOnce(() -> m_turret.setControl(m_MMVRequest.withPosition(rots)));
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        log_flywheelVelocityRPS.accept(m_flywheelLeader.getVelocity().getValueAsDouble());
        log_hoodPositionRots.accept(m_hood.getPosition().getValueAsDouble());
        log_turretPositionRots.accept(m_turret.getPosition().getValueAsDouble());

        log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(m_spunUp);
    }

    @Override
    public void simulationPeriodic() {
        // Shooter
        var leaderFXSim = m_flywheelLeader.getSimState();

        m_flywheelSim.setInputVoltage(leaderFXSim.getMotorVoltage());
        m_flywheelSim.update(Constants.kSimPeriodicUpdateInterval);

        leaderFXSim.setRotorVelocity(m_flywheelSim.getAngularVelocity());
        leaderFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Hood
        var hoodFXSim = m_hood.getSimState();

        m_hoodSim.setInputVoltage(hoodFXSim.getMotorVoltage());
        m_hoodSim.update(Constants.kSimPeriodicUpdateInterval);

        hoodFXSim.setRawRotorPosition(m_hoodSim.getAngleRads() / (2*Math.PI));
        hoodFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Turret
        var turretFXSim = m_turret.getSimState();

        m_turretSim.setInputVoltage(turretFXSim.getMotorVoltage());
        m_turretSim.update(Constants.kSimPeriodicUpdateInterval);

        turretFXSim.setRawRotorPosition(m_turretSim.getAngularPositionRotations() * ShooterK.kTurretGearing);
        turretFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

}