package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ShooterK.*;

import frc.robot.Constants;
import frc.util.MotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    // motors + control requests
    private final TalonFX m_flywheelLeader = new TalonFX(kLeaderCANID, Constants.kCanivoreBusName); //X60
    private final TalonFX m_flywheelFollower = new TalonFX(kFollowerCANID, Constants.kCanivoreBusName); //X60
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final Servo m_hood = new Servo(kHoodChannel);
    private final Canandmag m_hoodEncoder = new Canandmag(kHoodEncoderChannel);

    private final TalonFX m_turret = new TalonFX(kTurretCANID, Constants.kCanivoreBusName); //X44
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
            kFlywheelMoI,
            kFlywheelGearing
        ),
        DCMotor.getKrakenX60Foc(2) // returns gearbox
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
    private final DoubleLogger log_hoodPositionRots = WaltLogger.logDouble(kLogTab, "hoodPositionRots");    //logs the encoder values
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab, "turretPositionRots");

    private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    private final BooleanLogger log_hoodEncoderMagnetInRange = WaltLogger.logBoolean(kLogTab, "hoodEncoderMagnetInRange");
    private final BooleanLogger log_hoodEncoderPresent = WaltLogger.logBoolean(kLogTab, "hoodEncoderPresent");

    /* CONSTRUCTOR */
    public Shooter() {
        m_flywheelLeader.getConfigurator().apply(kFlywheelLeaderTalonFXConfiguration);
        m_flywheelFollower.getConfigurator().apply(kFlywheelFollowerTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);

        m_hoodEncoder.setSettings(kHoodEncoderSettings);    //if needed, we can add a position offset

        m_flywheelFollower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned

        initSim();
    }

    //TODO: update orientation values (if needed)
    private void initSim() {
        MotorSim.initSimFX(m_flywheelLeader, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX60);
        MotorSim.initSimFX(m_turret, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX44);
    }

    /* COMMANDS */
    // Shooter Commands (Veloity Control)
    public Command setFlywheelVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_flywheelLeader.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    // Hood Commands (Basic Position Control)
    public Command setHoodPositionCmd(Angle degs) {
        return runOnce(() -> m_hood.setAngle(degs.magnitude()));
    }

    // Turret Commands (Motionmagic Angle Control)
    public Command setTurretPositionCmd(Angle rots) {
        return runOnce(() -> m_turret.setControl(m_MMVRequest.withPosition(rots)));
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        log_flywheelVelocityRPS.accept(m_flywheelLeader.getVelocity().getValueAsDouble());
        log_hoodPositionRots.accept(m_hoodEncoder.getPosition());
        log_turretPositionRots.accept(m_turret.getPosition().getValueAsDouble());

        log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(m_spunUp);
        log_hoodEncoderMagnetInRange.accept(m_hoodEncoder.magnetInRange());
        log_hoodEncoderPresent.accept(m_hoodEncoder.isConnected());
    }

    @Override
    public void simulationPeriodic() {
        MotorSim.updateSimFX(m_flywheelLeader, m_flywheelSim);
        MotorSim.updateSimFX(m_turret, m_turretSim);
    }

}