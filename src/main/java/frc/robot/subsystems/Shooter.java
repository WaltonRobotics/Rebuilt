package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.ShooterK.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.TurretCalculator;
import frc.robot.subsystems.shooter.TurretVisualizer;
import frc.robot.subsystems.shooter.TurretCalculator.ShotData;
import frc.util.AllianceFlipUtil;
import frc.util.EqualsUtil;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

//TODO: for sohan - make sure to implement m_atGoal for Hood
public class Shooter extends SubsystemBase {
    /* VARIABLES */

    private Rotation2d m_goalAngle = Rotation2d.kZero;
    private double m_turretGoalVelocity = 0.0;
    private double m_turretLastGoalAngle = 0.0;
    private final double m_torqueCurrentControlTolerance = 20.0;
    private final double m_atGoalDebounce = 0.2;
    private double m_hoodLastGoalAngle = 0.0;
    private double m_hoodGoalAngle = 0.0;
    private double m_hoodGoalVelocity = 0.0;
    private double m_hoodOffset = 0.0;
    private double m_positionRad = 0;
    

    private ShootingState m_shootState = ShootingState.ACTIVE_SHOOTING;

    private boolean m_flywheelAtGoal, m_turretAtGoal, m_hoodAtGoal = false;
    private boolean m_turretZeroed = false;
    private boolean m_hoodZeroed = false;

    private State m_setpoint = new State();

    private Debouncer m_atGoalDebouncer = new Debouncer(m_atGoalDebounce, DebounceType.kFalling);

    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_fieldSpeedsSupplier;

    Translation3d currentTarget = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);

    private final TurretVisualizer m_turretVisualizer = new TurretVisualizer(
            () -> new Pose3d(RobotState.getInstance().getEstimatedPose()),
            () -> RobotState.getInstance().getRobotVelocity());

    // motors + control requests
    private final TalonFX m_leader = new TalonFX(kLeaderCANID); //X60
    private final TalonFX m_follower = new TalonFX(kFollowerCANID); //X60
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    private final TalonFX m_hood = new TalonFX(kHoodCANID); //X44

    private final PositionVoltage m_positionRequest = new PositionVoltage(0);

    private final TalonFX m_turret = new TalonFX(kTurretCANID); //X44
    private final MotionMagicVoltage m_MMVRequest = new MotionMagicVoltage(0);

    private final ShotCalculator shotCalulator;

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
        HoodPosition.HOME.rots * (2*Math.PI)
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
    private final DoubleLogger log_flywheelVelocityRPS = WaltLogger.logDouble(kLogTab, "flywheelVelocityRPS");
    private final DoubleLogger log_hoodPositionRots = WaltLogger.logDouble(kLogTab, "hoodPositionRots");
    private final DoubleLogger log_turretPositionRots = WaltLogger.logDouble(kLogTab, "turretPositionRots");

    private final BooleanLogger log_exitBeamBreak = WaltLogger.logBoolean(kLogTab, "exitBeamBreak");
    private final BooleanLogger log_spunUp = WaltLogger.logBoolean(kLogTab, "spunUp");

    /* CONSTRUCTOR */
    public Shooter(Swerve drivetrain, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        m_leader.getConfigurator().apply(kLeaderTalonFXConfiguration);
        m_follower.getConfigurator().apply(kFollowerTalonFXConfiguration);  //TODO: should the follower use the leader's configs?
        m_hood.getConfigurator().apply(kHoodTalonFXConfiguration);
        m_turret.getConfigurator().apply(kTurretTalonFXConfiguration);

        m_follower.setControl(new Follower(kLeaderCANID, MotorAlignmentValue.Opposed)); //TODO: check if MotorAlignmentValue is Opposed or Aligned

        m_poseSupplier = poseSupplier;
        m_fieldSpeedsSupplier = fieldSpeedsSupplier;

        shotCalulator = new ShotCalculator(drivetrain);

        initSim();
    }

    //sim stuff
    public boolean simAbleToIntake() {
        return m_turretVisualizer.canIntake();
    }

    public void simIntake() {
        m_turretVisualizer.intakeFuel();
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
    
    public void setShootState(ShootingState shootState) {
        this.m_shootState = shootState;
    }

    public void zeroTurret() {
        setTurretPositionCmd(TurretPosition.HOME);
        m_turretZeroed = true;
    }

    public Command zeroTurretCommmand() {
        return runOnce(this::zeroTurret).ignoringDisable(true);
    }

    public void zeroHood() {
        m_hoodOffset = kHoodMinAngle - Units.degreesToRadians(getHoodAngle());
        setHoodPositionCmd(HoodPosition.HOME);
        m_hoodZeroed = true;
    }

    public Command zeroHoodCommand() {
        return runOnce(this::zeroHood).ignoringDisable(true);
    }

    public Command zeroShooterCommand() {
        return Commands.sequence(
        zeroHoodCommand(),
        zeroTurretCommmand());
    }

    private void setFieldRelativeTarget(Rotation2d angle, double velocity) {
        m_goalAngle = angle;
        m_turretGoalVelocity = velocity;
    }

    public double getHoodAngle() {
        return m_hood.getPosition().getValue().in(Radians);
    }

    public boolean atGoal() {
        m_hoodAtGoal = 
            DriverStation.isEnabled()
                &&
                 m_hoodZeroed 
                    && Math.abs(getHoodAngle() - m_hoodGoalAngle)
                        <= 1.0; //degrees
        // m_hoodAtGoal = true;
        // m_turretAtGoal = true;
        // m_flywheelAtGoal = true;
        System.out.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" + (m_hoodAtGoal && m_turretAtGoal
                && m_flywheelAtGoal) + "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

        return m_hoodAtGoal && m_turretAtGoal && m_flywheelAtGoal;
    }

    private void setHoodGoalParams(double angle, double velocity) {
        m_hoodGoalAngle = angle;
        m_hoodGoalVelocity = velocity;
    }


    /* COMMANDS */
    // Shooter Commands (Velocity Control)
    public Command setFlywheelVelocityCmd(FlywheelVelocity velocity) {
        return setFlywheelVelocityCmd(velocity.RPS);
    }

    public Command setFlywheelVelocityCmd(double RPS) {
        boolean inTolerance = RPS <= m_torqueCurrentControlTolerance;
        m_flywheelAtGoal = m_atGoalDebouncer.calculate(inTolerance);

        return runOnce(() -> m_leader.setControl(m_velocityRequest.withVelocity(RPS)));
    }

    private void stopFlywheel() {
        m_leader.setNeutralMode(NeutralModeValue.Coast);
        setFlywheelVelocityCmd(0);
        m_flywheelAtGoal = false;
    }

    public Command runFlywheelFixedCommand(DoubleSupplier velocity) {
        return runEnd(() -> setFlywheelVelocityCmd(velocity.getAsDouble()), this::stopFlywheel);
    }

    // Hood Commands (Basic Position Control)
    public Command setHoodPositionCmd(HoodPosition position) {
        return setHoodPositionCmd(position.rots);
    }

    public Command setHoodPositionCmd(double rots) {
        return runOnce(() -> m_hood.setControl(m_positionRequest.withPosition(rots)));
    }

    public Command runHoodFixedCommand(DoubleSupplier angle, DoubleSupplier velocity) {
        return run(() -> setHoodGoalParams(angle.getAsDouble(), velocity.getAsDouble()));
    }

    // Turret Commands (Motionmagic Angle Control)
    public Command setTurretPositionCmd(TurretPosition position) {
        return setTurretPositionCmd(position.rots);
    }

    public Command setTurretPositionCmd(double rots) {
        return runOnce(() -> m_turret.setControl(m_MMVRequest.withPosition(rots)));
    }

    public double getTurretPosition() {
        return m_turret.getPosition().getValueAsDouble();
    }

    public Command runTurretTrackTargetCommand() {
        return run(
            () -> {
                var params = shotCalulator.getParameters();
                setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
                setShootState(ShootingState.TRACKING);
            }
        );
    }

    public Command runHoodTrackTargetCommand() {
        return run(
            () -> {
                var params = shotCalulator.getParameters();
                setHoodGoalParams(params.hoodAngle(), params.turretVelocity());
            }
        );
    }

    public Command runFlywheelTrackTargetCommand() {
        return runEnd(
            () -> setFlywheelVelocityCmd(shotCalulator.getParameters().flywheelSpeed()), 
            this::stopFlywheel);
    }

    public Command runTurretTrackTargetActiveShootingCommand() {
        return run(
            () -> {
                var params = shotCalulator.getParameters();
                atGoal();
                setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
                setShootState(ShootingState.ACTIVE_SHOOTING);
            });
    }

    private void calculateShot() {
        Pose2d robot = m_poseSupplier.get();
        ChassisSpeeds fieldSpeeds = m_fieldSpeedsSupplier.get();

        ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(robot, fieldSpeeds, null , kExitBeamBreakChannel)
    }

    public Command shooterDefaultCommands() {
        return run (
            () -> {
                runTurretTrackTargetCommand();
                runHoodTrackTargetCommand();
                runFlywheelTrackTargetCommand();
            });
    }
 
    /* PERIODICS */
    @Override
    public void periodic() {
        if (DriverStation.isDisabled() || !m_turretZeroed) {
            m_setpoint = new State(m_turretSim.getAngularPositionRad(), 0.0);
            m_turretLastGoalAngle = getTurretPosition();
            m_turretAtGoal = false;
        } else if (DriverStation.isEnabled() && m_turretZeroed) {
            Rotation2d robotAngle = RobotState.getInstance().getRotation();
            Rotation2d robotRelativeGoalAngle = m_goalAngle.minus(robotAngle);
            boolean hasBestAngle = false;
            double bestAngle = 0;
            double minLegalAngle = switch (m_shootState) {
                case ACTIVE_SHOOTING -> kTurretMinAngle;
                case TRACKING -> kTurretMaxAngle;
            };
            double maxLegalAngle = switch (m_shootState) {
                case ACTIVE_SHOOTING -> kTurretMaxAngle;
                case TRACKING -> kTurretMinAngle;
            };
            double robotAngularVelocity = RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;
            double robotRelativeGoalVelocity = m_turretGoalVelocity - robotAngularVelocity;

            for (int i = -2; i < 3; i++) {
                double potentialSetpoint = robotRelativeGoalAngle.getRadians() + Math.PI * 2.0 * i;

                if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
                    continue;
                } else {
                    if (!hasBestAngle) {
                        bestAngle = potentialSetpoint;
                        hasBestAngle = true;
                    }
                    if (Math.abs(m_turretLastGoalAngle - potentialSetpoint) < Math.abs(m_turretLastGoalAngle - bestAngle)) {
                        bestAngle = potentialSetpoint;
                    }
                }
            }
            
            m_turretAtGoal =
                EqualsUtil.epsilonEquals(bestAngle, m_setpoint.position)
                    && EqualsUtil.epsilonEquals(robotRelativeGoalVelocity, m_setpoint.velocity);
        }

        if (DriverStation.isEnabled() && m_hoodZeroed) {
            m_positionRad = MathUtil.clamp(m_hoodGoalAngle, kHoodMinAngle, kHoodMaxAngle);
            m_hoodGoalVelocity = m_hoodGoalVelocity;
        }
        
        log_flywheelVelocityRPS.accept(m_leader.getVelocity().getValueAsDouble());
        log_hoodPositionRots.accept(m_hood.getPosition().getValueAsDouble());
        log_turretPositionRots.accept(m_turret.getPosition().getValueAsDouble());

        log_exitBeamBreak.accept(trg_exitBeamBreak);
        log_spunUp.accept(m_spunUp);
    }

    @Override
    public void simulationPeriodic() {
        // Flywheel
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
    public enum FlywheelVelocity {
        // in RPS
        ZERO(0),
        SCORE(5.5),
        PASS(7),
        MAX(20);

        public double RPS;
        private FlywheelVelocity(double RPS) {
            this.RPS = RPS;
        }
    }

    public enum HoodPosition {
        MIN(0),
        HOME(5),
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