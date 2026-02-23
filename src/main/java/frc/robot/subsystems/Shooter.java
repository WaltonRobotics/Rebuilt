package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.util.WaltMotorSim;
import frc.util.GobildaServoContinuous;
import frc.util.GobildaServoAngled;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    /* CLASS VARIABLES */
    //---MOTORS + CONTROL REQUESTS
    private final TalonFX m_shooterLeader = new TalonFX(kLeaderCANID, Constants.kCanivoreBus); //X60Foc
    private final TalonFX m_shooterFollower = new TalonFX(kFollowerCANID, Constants.kCanivoreBus); //X60Foc
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final TalonFX m_turret = new TalonFX(kTurretCANID, Constants.kCanivoreBus); //X44Foc
    private final MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    private final GobildaServoAngled m_hood = new GobildaServoAngled(kHoodChannel);
    private final CANcoder m_hoodEncoder = new CANcoder(kHoodEncoderCANID, Constants.kCanivoreBus);

    private Angle m_hoodSetpoint = Degrees.of(0);
    private Angle m_currentHoodPos = Rotations.of(0);

    //---LOGIC BOOLEANS
    private boolean m_spunUp = false;   // currently unused
    private final boolean m_inSim = RobotBase.isSimulation();

    //---BEAM BREAKS (if we have one on the shooter)
    // public DigitalInput m_exitBeamBreak = new DigitalInput(kExitBeamBreakChannel);
    // public final Trigger trg_exitBeamBreak = new Trigger(() -> !m_exitBeamBreak.get()); //true when beam is broken

    // public final Trigger exitBeamBreakTrigger(EventLoop loop) {
    //     return new Trigger(loop, () -> !m_exitBeamBreak.get());
    // }

    /* SIM OBJECTS */
    private final FlywheelSim m_shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(2), 
            kShooterMoI,
            kShooterGearing
        ),
        DCMotor.getKrakenX60Foc(2) // returns gearbox
    );

    // Since the servo acts like a DC Motor, we use DCMotorSim
    private final DCMotorSim m_hoodSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            khoodDCMotorGearbox,
            kHoodMoI,
            kHoodGearing
        ),
        khoodDCMotorGearbox // returns gearbox
    );

    private final DCMotorSim m_turretSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1),
            kTurretMoI,
            kTurretGearing
        ),
        DCMotor.getKrakenX44Foc(1) // returns gearbox
    );

    /* LOGGERS */
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble(kLogTab, "shooterVelocityRPS");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab, "turretPositionRots");

    //---HOOD
    private final DoubleLogger log_hoodEncoderPositionDegs = WaltLogger.logDouble(kLogTab, "hoodEncoderPositionDegs");
    private final DoubleLogger log_hoodEncoderVelocityRPS = WaltLogger.logDouble(kLogTab, "hoodEncoderVelocityRPS");
    private final DoubleLogger log_hoodReferencePosition = WaltLogger.logDouble(kLogTab, "hoodReferencePosition");
    private final DoubleLogger log_hoodEncoderError = WaltLogger.logDouble(kLogTab, "hoodEncoderError");

    // private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    private final DoubleLogger log_hoodServoVoltage = WaltLogger.logDouble(kLogTab, "hoodServoVoltage");
    private final DoubleLogger log_hoodServoCurrent = WaltLogger.logDouble(kLogTab, "hoodServoCurrent");

    private final BooleanLogger log_isHoodHoming = WaltLogger.logBoolean(kLogTab, "isHoodHoming");

    private boolean m_isHoodHoming = false;
    private double referenceRPS = 0;

    private Timer m_timer = new Timer();

    /* CONSTRUCTOR */
    public Shooter() {
        m_shooterLeader.getConfigurator().apply(kShooterLeaderTalonFXConfiguration);
        m_shooterFollower.getConfigurator().apply(kShooterFollowerTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);    
        m_hoodEncoder.getConfigurator().apply(kHoodEncoderConfiguration);    //if needed, we can add a position offset

        m_shooterFollower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned

        if(Robot.isReal()) {
            setDefaultCommand(hoodEncoderHoming());
        }

        // SmartDashboard.putData(m_hoodPIDUp);
        // SmartDashboard.putData(m_hoodPIDDown);

        initSim();
    }

    //TODO: update orientation values (if needed)
    private void initSim() {
        WaltMotorSim.initSimFX(m_shooterLeader, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX60);
        WaltMotorSim.initSimFX(m_turret, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX44);
    }

    /* COMMANDS */
    // ---SHOOTER (Veloity Control)
    public void setShooterVelocity(AngularVelocity RPS) {
        m_shooterLeader.setControl(m_velocityRequest.withVelocity(RPS));
    }

    public Command setShooterVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_shooterLeader.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    //for TestingDashboard
    public Command setShooterVelocityCmd(DoubleSubscriber sub_RPS) {
        return run(() -> m_shooterLeader.setControl(m_velocityRequest.withVelocity(RotationsPerSecond.of(sub_RPS.get()))));
    }

    public boolean checkIfSpunUp(Double RPS) {
        referenceRPS = RPS;
        if (m_shooterLeader.getVelocity().getValueAsDouble() > RPS - (RPS * 0.05)) {
            return true;
        }
        return false;
    }

    //---HOOD (Basic Position Control)
    public Command setHoodPositionCmd(Angle degs) {
        m_hoodSetpoint = degs;
        return runOnce(() -> m_hood.setAngle(degs.times(kHoodGearing).magnitude()));
    }

    public void setHoodPosition(Angle degs) {
        m_hoodSetpoint = degs;
        m_hood.setAngle(degs.times(kHoodGearing).magnitude());
    }

    //for TestingDashboard
    public Command setHoodPositionCmd(DoubleSubscriber sub_degs) {
        m_hoodSetpoint = Degrees.of(sub_degs.getAsDouble());
        return run(() -> m_hood.setAngle(sub_degs.getAsDouble() * kHoodGearing));
    }

    public Command hoodEncoderHoming() {
        Runnable init = () -> {
            m_timer.start();
            m_hood.setAngle(0);
        };

        Runnable execute = () -> {};

        Consumer<Boolean> onEnd = (Boolean interrupted) -> {
            m_hoodEncoder.setPosition(Rotations.of(0));
            m_timer.stop();
            m_timer.reset();
            removeDefaultCommand();
        };

        BooleanSupplier isFinished = () -> 
            ((Math.abs(m_hoodEncoder.getVelocity().getValueAsDouble()) < 0.001) && (m_timer.get() / 1000 > 350));

        return new FunctionalCommand(init, execute, onEnd, isFinished, this).withTimeout(3).withName("hoodEncoder homing");
    }

    public Command setHoodEncoderZero() {
        return Commands.runOnce(() -> m_hoodEncoder.setPosition(0));
    }

    //---TURRET (Motionmagic Angle Control)
    public Command setTurretPositionCmd(Angle rots) {
        return runOnce(() -> m_turret.setControl(m_MMVRequest.withPosition(rots)));
    }

    //for TestingDashboard
    public Command setTurretPositionCmd(DoubleSubscriber sub_rots) {
        return run(() -> m_turret.setControl(m_MMVRequest.withPosition(Rotations.of(sub_rots.get()))));
    }

    /* GETTERS */
    public TalonFX getShooter() {
        return m_shooterLeader;
    }

    public DCMotorSim getHoodSimEncoder() {
        return m_hoodSim;
    }

    public TalonFX getTurret() {
        return m_turret;
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        //---Loggers
        log_shooterVelocityRPS.accept(m_shooterLeader.getVelocity().getValueAsDouble());
        log_hoodEncoderPositionDegs.accept(Rotations.of(m_hoodEncoder.getPosition().getValueAsDouble()).in(Degrees) / kHoodGearing);
        log_hoodEncoderVelocityRPS.accept(m_hoodEncoder.getVelocity().getValueAsDouble());
        log_hoodReferencePosition.accept(m_hood.getAngle() / kHoodGearing);
        log_hoodEncoderError.accept(Math.abs((m_hoodSetpoint.in(Degrees)) - (Rotations.of(m_hoodEncoder.getPosition().getValueAsDouble()).in(Degrees))) / kHoodGearing);
        log_isHoodHoming.accept(m_isHoodHoming);

        log_turretPositionRots.accept(m_turret.getPosition().getValueAsDouble());

        // log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(checkIfSpunUp(referenceRPS));
        log_hoodServoVoltage.accept(RobotController.getVoltage6V());
        log_hoodServoCurrent.accept(RobotController.getCurrent6V());
    }

    @Override
    public void simulationPeriodic() {
        WaltMotorSim.updateSimFX(m_shooterLeader, m_shooterSim);
        WaltMotorSim.updateSimFX(m_turret, m_turretSim);
        // WaltMotorSim.updateSimServo(m_hood, m_hoodSim);  //works only w continuous for now
    }

}