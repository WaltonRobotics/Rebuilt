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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ShooterK.*;

import frc.robot.Constants;
import frc.util.MotorSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    // motors + control requests
    private final TalonFX m_shooterLeader = new TalonFX(kLeaderCANID, Constants.kCanivoreBus); //X60
    private final TalonFX m_shooterFollower = new TalonFX(kFollowerCANID, Constants.kCanivoreBus); //X60
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final Servo m_hood = new Servo(kHoodChannel);
    private final Canandmag m_hoodEncoder = new Canandmag(kHoodEncoderChannel);
    private final PIDController m_hoodPID = new PIDController(1, 0, 0);

    private Angle m_hoodSetpoint = Degrees.of(0);
    private Angle m_currentHoodPos = Rotations.of(0);

    private final TalonFX m_turret = new TalonFX(kTurretCANID, Constants.kCanivoreBus); //X44
    private final MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    // logic booleans
    private boolean m_spunUp = false;   //unused
    private final boolean m_inSim = RobotBase.isSimulation();

    // UNUSED - beam breaks (if we have one on the shooter)
    public DigitalInput m_exitBeamBreak = new DigitalInput(kExitBeamBreakChannel);

    public final Trigger trg_exitBeamBreak = new Trigger(() -> !m_exitBeamBreak.get()); //true when beam is broken

    public final Trigger exitBeamBreakTrigger(EventLoop loop) {
        return new Trigger(loop, () -> !m_exitBeamBreak.get());
    }

    // sim (TODO: double check constants)
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

    // loggers
    private final DoubleLogger log_shooterVelocityRPS = WaltLogger.logDouble(kLogTab, "shooterVelocityRPS");
    private final DoubleLogger log_hoodPositionDegs = WaltLogger.logDouble(kLogTab, "hoodPositionDegs");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab, "turretPositionRots");

    private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    private final BooleanLogger log_hoodEncoderMagnetInRange = WaltLogger.logBoolean(kLogTab, "hoodEncoderMagnetInRange");
    private final BooleanLogger log_hoodEncoderPresent = WaltLogger.logBoolean(kLogTab, "hoodEncoderPresent");

    private final DoubleLogger log_hoodPIDOutput = WaltLogger.logDouble(kLogTab, "hoodPIDOutput");

    /* CONSTRUCTOR */
    public Shooter() {
        m_shooterLeader.getConfigurator().apply(kShooterLeaderTalonFXConfiguration);
        m_shooterFollower.getConfigurator().apply(kShooterFollowerTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);

        m_hoodEncoder.setSettings(kHoodEncoderSettings);    //if needed, we can add a position offset

        m_shooterFollower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned

        initSim();
    }

    //TODO: update orientation values (if needed)
    private void initSim() {
        MotorSim.initSimFX(m_shooterLeader, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX60);
        MotorSim.initSimFX(m_turret, ChassisReference.CounterClockwise_Positive, TalonFXSimState.MotorType.KrakenX44);
    }

    /* COMMANDS */
    // Shooter Commands (Veloity Control)
    public Command setShooterVelocityCmd(AngularVelocity RPS) {
        return runOnce(() -> m_shooterLeader.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    // Hood Commands (Basic Position Control)
    public Command setHoodPositionCmd(Angle degs) {  
        return runOnce(
            () -> m_hoodSetpoint = degs
        );
    }

    // The PIDOutput needed to get to the setpoint from the current point
    public void updateHood() {
        // can't read from hardware in Sim, so we read from the hoodSim object
        m_currentHoodPos = m_inSim ? Rotations.of(m_hoodSim.getAngularPositionRotations()) : Rotations.of(m_hoodEncoder.getPosition());

        double hoodPIDOutput = m_hoodPID.calculate(m_currentHoodPos.magnitude(), m_hoodSetpoint.in(Rotations)) * 8;
        
        hoodPIDOutput = MathUtil.clamp(hoodPIDOutput, -1.0, 1.0);
        hoodPIDOutput = (hoodPIDOutput + 1) / 2;
        m_hood.set(hoodPIDOutput);

        log_hoodPIDOutput.accept(hoodPIDOutput);
    }

    // Turret Commands (Motionmagic Angle Control)
    public Command setTurretPositionCmd(Angle rots) {
        return runOnce(() -> m_turret.setControl(m_MMVRequest.withPosition(rots)));
    }

    public TalonFX getShooter() {
        return m_shooterLeader;
    }

    public DCMotorSim getSimHoodEncoder() {
        return m_hoodSim;
    }

    public TalonFX getTurret() {
        return m_turret;
    }

    /* PERIODICS */
    @Override
    public void periodic() {
        //---Hood
        updateHood();

        //---Loggers
        log_shooterVelocityRPS.accept(m_shooterLeader.getVelocity().getValueAsDouble());
        log_hoodPositionDegs.accept(m_currentHoodPos.in(Degrees));
        log_turretPositionRots.accept(m_turret.getPosition().getValueAsDouble());

        log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(m_spunUp);
        log_hoodEncoderMagnetInRange.accept(m_hoodEncoder.magnetInRange());
        log_hoodEncoderPresent.accept(m_hoodEncoder.isConnected());
    }

    @Override
    public void simulationPeriodic() {
        MotorSim.updateSimFX(m_shooterLeader, m_shooterSim);
        MotorSim.updateSimFX(m_turret, m_turretSim);
        MotorSim.updateSimServo(m_hood, m_hoodSim);
    }

}