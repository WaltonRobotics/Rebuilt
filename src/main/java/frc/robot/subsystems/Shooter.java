package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.shooter.ShotCalculation;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

//TODO: for sohan - make sure to implement m_atGoal for turret and Hood (flywheel alr done)
public class Shooter extends SubsystemBase {
    /* VARIABLES */

    private Rotation2d m_goalAngle = Rotation2d.kZero;
    private double m_goalVelocity = 0.0;
    private double m_lastGoalAngle = 0.0;
    private ShootingState m_shootState = ShootingState.ACTIVE_SHOOTING;
    private boolean m_shooterAtGoal, m_turretAtGoal, m_hoodAtGoal = false;
    private static final double m_torqueCurrentControlTolerance = 20.0;
    private static final double m_atGoalDebounce = 0.2;

    private Debouncer m_atGoalDebouncer = new Debouncer(m_atGoalDebounce, DebounceType.kFalling);


    // motors + control requests
    private final TalonFX m_leader = new TalonFX(kLeaderCANID); //X60
    private final TalonFX m_follower = new TalonFX(kFollowerCANID); //X60
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    private final TalonFX m_hood = new TalonFX(kHoodCANID); //X44
    private final PositionVoltage m_positionRequest = new PositionVoltage(0);

    private final TalonFX m_turret = new TalonFX(kTurretCANID); //X44
    private final MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0);

    private final ShotCalculation shotCalulator = new ShotCalculation();

    // logic booleans
    private boolean m_spunUp = false;

    // beam breaks (if we have one on the shooter)
    public DigitalInput m_exitBeamBreak = new DigitalInput(kExitBeamBreakChannel);

    public final Trigger trg_exitBeamBreak = new Trigger(() -> !m_exitBeamBreak.get()); //true when beam is broken

    public final Trigger exitBeamBreakTrigger(EventLoop loop) {
        return new Trigger(loop, () -> !m_exitBeamBreak.get());
    }

    // sim (TODO: Update dummy numbers)
    private final FlywheelSim m_flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2), 
            0.000349, // J for 2 3" 0.53lb flywheels (double check accuracy)
            1), // TODO: dummy value
        DCMotor.getKrakenX60(2) // returns gearbox
    );

    private final SingleJointedArmSim m_hoodSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX44(1),
            0.0005,  // Dummy J Value
            1.5 // dummy gearing value
        ),
        DCMotor.getKrakenX44(1),
        1.5,   //dummy gearing value
        0.5,
        HoodPosition.MIN.rots * (2*Math.PI),
        HoodPosition.MAX.rots * (2*Math.PI),
        false,
        HoodPosition.INIT.rots * (2*Math.PI)
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1),
            0.0005,  // Dummy J value
            1.5 // dummy gearing value
        ),
        DCMotor.getKrakenX44(1) // returns gearbox
    );

    // loggers
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble(kLogTab, "shooterVelocityRPS");
    private final DoubleLogger log_hoodPositionRots = WaltLogger.logDouble(kLogTab, "hoodPositionRots");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab, "turretPositionRots");

    private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    /* CONSTRUCTOR */
    public Shooter() {
        m_leader.getConfigurator().apply(kLeaderTalonFXConfiguration);
        m_follower.getConfigurator().apply(kFollowerTalonFXConfiguration);  //TODO: should the follower use the leader's configs?
        m_hood.getConfigurator().apply(kHoodTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);

        m_follower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned

        initSim();
    }

    //TODO: update orientation values (if needed)
    private void initSim() {
        var m_leaderFXSim = m_leader.getSimState();
        m_leaderFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        m_leaderFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        var m_hoodFXSim = m_hood.getSimState();
        m_hoodFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        m_hoodFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        var m_turretFXSim = m_turret.getSimState();
        m_turretFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        m_turretFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }
    
    public Rotation2d getGoalAngle() {
        return m_goalAngle;
    }
    public void setGoalAngle(Rotation2d goalAngle) {
        this.m_goalAngle = goalAngle;
    }
    public double getGoalVelocity() {
        return m_goalVelocity;
    }
    public void setGoalVelocity(double goalVelocity) {
        this.m_goalVelocity = goalVelocity;
    }
    public double getLastGoalAngle() {
        return m_lastGoalAngle;
    }
    public void setLastGoalAngle(double lastGoalAngle) {
        this.m_lastGoalAngle = lastGoalAngle;
    }

    private void setFieldRelativeTarget(Rotation2d angle, double velocity) {
        this.m_goalAngle = angle;
        this.m_goalVelocity = velocity;
    }

    public ShootingState getShootState() {
        return m_shootState;
    }

    public void setShootState(ShootingState shootState) {
        this.m_shootState = shootState;
    }

    /* COMMANDS */
    // Shooter Commands (Velocity Control)
    public Command setShooterVelocityCmd(ShooterVelocity velocity) {
        return setShooterVelocityCmd(velocity.RPS);
    }

    public Command setShooterVelocityCmd(double RPS) {
        boolean inTolerance = RPS <= m_torqueCurrentControlTolerance;
        m_shooterAtGoal = m_atGoalDebouncer.calculate(inTolerance);

        return runOnce(() -> m_leader.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    // Hood Commands (Basic Position Control)
    public Command setHoodPositionCmd(HoodPosition position) {
        return setHoodPositionCmd(position.rots);
    }

    public Command setHoodPositionCmd(double rots) {
        return runOnce(() -> m_hood.setControl(m_positionRequest.withPosition(rots)));
    }

    // Turret Commands (Motionmagic Angle Control)
    public Command setTurretPositionCmd(TurretPosition position) {
        return setTurretPositionCmd(position.rots);
    }

    public Command setTurretPositionCmd(double rots) {
        return runOnce(() -> m_turret.setControl(m_MMVRequest.withPosition(rots)));
    }

    public Command runTrackTargetActiveShootingCommand() {
        return run(
            () -> {
                var params = shotCalulator.getParameters();
                setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
                setShootState(ShootingState.TRACKING);
            });
    }
 
    /* PERIODICS */
    @Override
    public void periodic() {
        log_shooterVelocityRPS.accept(m_leader.getVelocity().getValueAsDouble());
        log_hoodPositionRots.accept(m_hood.getPosition().getValueAsDouble());
        log_turretPositionRots.accept(m_turret.getPosition().getValueAsDouble());

        log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(m_spunUp);
    }

    @Override
    public void simulationPeriodic() {
        // Shooter
        var m_leaderFXSim = m_leader.getSimState();

        m_flywheelSim.setInputVoltage(m_leaderFXSim.getMotorVoltage());
        m_flywheelSim.update(Constants.kSimPeriodicUpdateInterval);

        m_leaderFXSim.setRotorVelocity(m_flywheelSim.getAngularVelocityRPM() / 60);
        m_leaderFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Hood
        var m_hoodFXSim = m_hood.getSimState();

        m_hoodSim.setInputVoltage(m_hoodFXSim.getMotorVoltage());
        m_hoodSim.update(Constants.kSimPeriodicUpdateInterval);

        m_hoodFXSim.setRawRotorPosition(m_hoodSim.getAngleRads() / (2*Math.PI));
        m_hoodFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Turret
        var m_turretFXSim = m_turret.getSimState();

        m_turretSim.setInputVoltage(m_turretFXSim.getMotorVoltage());
        m_turretSim.update(Constants.kSimPeriodicUpdateInterval);

        m_turretFXSim.setRawRotorPosition(m_turretSim.getAngularPositionRotations());
        m_turretFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    /* CONSTANTS */
    public enum ShooterVelocity {
        // in RPS
        ZERO(0),
        SCORE(5.5),
        PASS(7),
        MAX(20);

        public double RPS;
        private ShooterVelocity(double RPS) {
            this.RPS = RPS;
        }
    }

    public enum HoodPosition {
        MIN(0),
        INIT(5),
        SCORE(10),
        PASS(20),
        MAX(40);

        public double rots;
        private HoodPosition(double rots) {
            this.rots = rots;
        }
    }

    public enum TurretPosition {
        MIN(0),
        HOME(10),
        SCORE(20),
        PASS(30),
        MAX(40);

        public double rots;
        private TurretPosition(double rots) {
            this.rots = rots;
        }
    }

    public enum ShootingState {
        ACTIVE_SHOOTING,
        TRACKING;
    }

}