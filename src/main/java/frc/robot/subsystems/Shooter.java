package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ShooterK.*;

import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Shooter extends SubsystemBase {
    /* VARIABLES */
    // motors + control requests
    private final TalonFX m_leader = new TalonFX(kLeaderCANID); //X60
    private final TalonFX m_follower = new TalonFX(kFollowerCANID); //X60
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    private final TalonFX m_hood = new TalonFX(kHoodCANID); //X44
    private final PositionVoltage m_positionRequest = new PositionVoltage(0);

    private final TalonFX m_turret = new TalonFX(kTurretCANID); //X44

    // speeds
    private final double m_passSpeed = 7.0; //TODO: Update Speeds
    private final double m_scoreSpeed = 5.5;

    // logic booleans
    private boolean m_spunUp = false;

    // beam breaks (if needed)
    public DigitalInput m_exitBeamBreak = new DigitalInput(kExitBeamBreakChannel);

    public final Trigger trg_exitBeamBreak = new Trigger(() -> !m_exitBeamBreak.get()); //true when beam is broken

    public final Trigger exitBeamBreakTrigger(EventLoop loop) {
        return new Trigger(loop, () -> !m_exitBeamBreak.get());
    }

    // loggers
    private final DoubleLogger log_shooterRPM = WaltLogger.logDouble(kLogTab, "shooterRPM");
    private final DoubleLogger log_hoodPosition = WaltLogger.logDouble(kLogTab, "hoodPosition");

    private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    /* CONSTRUCTOR */
    public Shooter() {
        m_leader.getConfigurator().apply(kLeaderTalonFXConfiguration);
        m_follower.getConfigurator().apply(kFollowerTalonFXConfiguration);
        m_hood.getConfigurator().apply(kHoodTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);

        m_follower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned
    }

    /* COMMANDS */
    // Shooter Commands (Veloity Control)
    public void setVelocity(double desiredVelocity) {
        m_leader.setControl(m_velocityRequest.withVelocity(desiredVelocity));
    }

    public Command setVelocityCmd(double desiredVelocity) {
        return runOnce(() -> setVelocity(desiredVelocity));
    }

    public Command score() {
        return setVelocityCmd(m_scoreSpeed);
    }

    public Command pass() {
        return setVelocityCmd(m_passSpeed);
    }

    // Hood Commands (Basic Position Control)
    public void setPosition(double desiredPosition) {
        m_hood.setControl(m_positionRequest.withPosition(desiredPosition));
    }

    public Command setPositionCmd(double desiredPosition) {
        return runOnce(() -> setPosition(desiredPosition));
    }

    // Turret Commands (Motionmagic Angle Control); Saarth will work on this via "2021-Gamechangers" code


    @Override
    public void periodic() {
        log_shooterRPM.accept(m_leader.getVelocity().getValueAsDouble());
        log_hoodPosition.accept(m_hood.getPosition().getValueAsDouble());

        log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(m_spunUp);
    }

}
