// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IndexerK;
import static frc.robot.Constants.ShooterK;
import static frc.robot.Constants.RobotK.*;

import java.util.HashMap;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeK;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.VisionK;
import frc.robot.autons.AutonChooser;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.Intake.IntakeRollersVelocity;
import frc.robot.subsystems.Indexer;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Robot extends TimedRobot {
    private final double kMaxTranslationSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final double kMaxHighAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    private final DoubleLogger log_stickDesiredFieldX = WaltLogger.logDouble("Swerve", "stick desired teleop x");
    private final DoubleLogger log_stickDesiredFieldY = WaltLogger.logDouble("Swerve", "stick desired teleop y");
    private final DoubleLogger log_stickDesiredFieldZRot = WaltLogger.logDouble("Swerve", "stick desired teleop z rot");
    private final BooleanLogger log_povUp = WaltLogger.logBoolean(kLogTab, "Pov Up");
    private final BooleanLogger log_povRight = WaltLogger.logBoolean(kLogTab, "Pov Right");
    private final BooleanLogger log_povLeft = WaltLogger.logBoolean(kLogTab, "Pov Left");
    private final BooleanLogger log_povDown = WaltLogger.logBoolean(kLogTab, "Pov Down");

    private double m_visionSeenLastSec = Utils.getCurrentTimeSeconds();
    private final BooleanLogger log_visionSeenPastSecond = new BooleanLogger(kLogTab, "VisionSeenLastSec");

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_manipulator = new CommandXboxController(1);

    public final Swerve m_drivetrain = TunerConstants.createDrivetrain();

    private Command m_autonomousCommand;
    private String m_autonChosen = "noAutonSelected";

    private final AutoFactory m_autoFactory = m_drivetrain.createAutoFactory();
    private final WaltAutonFactory m_waltAutonFactory = new WaltAutonFactory(m_autoFactory, m_drivetrain);
    private HashMap<String, Command> m_autonList = new HashMap<String, Command>();

    private final VisionSim m_visionSim = new VisionSim();

    // this should be updated with all of our cameras
    private final Vision[] m_cameras = {
        new Vision(VisionK.kCameras[0], m_visionSim),
        new Vision(VisionK.kCameras[1], m_visionSim),
        new Vision(VisionK.kCameras[2], m_visionSim),
        new Vision(VisionK.kCameras[3], m_visionSim),
    };

    private final Shooter m_shooter = new Shooter();
    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();

    private final Superstructure m_superstructure = new Superstructure(m_intake, m_indexer, m_shooter);

    private Trigger trg_swerveToObject = m_driver.x();

    private Trigger trg_driverOverride = m_driver.b();
    private Trigger trg_manipOverride = m_manipulator.b();

    // Command sequence triggers
    private Trigger trg_activateIntake = m_manipulator.a().and(trg_manipOverride.negate());
    private Trigger trg_prepIntake = m_manipulator.x().and(trg_manipOverride.negate());
    private Trigger trg_retractIntake = m_manipulator.y().and(trg_manipOverride.negate());

    private Trigger trg_shoot = m_driver.rightTrigger().and(trg_manipOverride.negate());
    private Trigger trg_emergencyBarf = m_driver.leftTrigger().and(trg_manipOverride.negate());

    private Trigger trg_pass = m_driver.rightBumper().and(trg_manipOverride.negate());

    // Override triggers
    private Trigger trg_maxShooterOverride = trg_manipOverride.and(m_manipulator.povLeft());

    private Trigger trg_turret180Override = trg_manipOverride.and(m_manipulator.povRight());

    private Trigger trg_hood30Override = trg_manipOverride.and(m_manipulator.povUp());

    private Trigger trg_startSpindexerOverride = trg_manipOverride.and(m_manipulator.rightBumper());

    private Trigger trg_startTunnelOverride = trg_manipOverride.and(m_manipulator.leftBumper());

    private Trigger trg_maxRollersOverride = trg_manipOverride.and(m_manipulator.povDown());

    private Trigger trg_deployIntakeOverride = trg_manipOverride.and(m_manipulator.rightTrigger());
    private Trigger trg_intakeUpOverride = trg_manipOverride.and(m_manipulator.leftTrigger());

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        configureBindings();
        // configureTestBindings();    //this should be commented out during competition matches
    }

    private Command driveCommand() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will execute this command periodically
        return m_drivetrain.applyRequest(() -> {
            var angularRate = m_driver.leftTrigger().getAsBoolean() ? 
            kMaxHighAngularRate : kMaxAngularRate;
        
            var driverXVelo = -m_driver.getLeftY() * kMaxTranslationSpeed;
            var driverYVelo = -m_driver.getLeftX() * kMaxTranslationSpeed;
            var driverYawRate = -m_driver.getRightX() * angularRate;

            log_stickDesiredFieldX.accept(driverXVelo);
            log_stickDesiredFieldY.accept(driverYVelo);
            log_stickDesiredFieldZRot.accept(driverYawRate);
            
            return drive
            .withVelocityX(driverXVelo) // Drive forward with Y (forward)
            .withVelocityY(driverYVelo) // Drive left with X (left)
            .withRotationalRate(driverYawRate); // Drive counterclockwise with negative X (left)
            }
        );
    }

    private void configureBindings() {
        /* GENERATED SWERVE BINDS */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // m_driver.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
        // m_driver.b().whileTrue(m_drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_driver.getLeftY(), -m_driver.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driver.back().and(m_driver.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        // m_driver.back().and(m_driver.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driver.start().and(m_driver.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driver.start().and(m_driver.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        m_driver.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        m_drivetrain.registerTelemetry(logger::telemeterize);

        /* CUSTOM BINDS */

        //robot heads toward fuel when detected :D (hypothetically)(robo could blow up instead)
        trg_swerveToObject.whileTrue(m_drivetrain.swerveToObject());

        // Test sequences
        trg_activateIntake.onTrue(m_superstructure.activateIntake());
        trg_prepIntake.onTrue(m_superstructure.deactivateIntake(IntakeArmPosition.SAFE));
        trg_retractIntake.onTrue(m_superstructure.deactivateIntake(IntakeArmPosition.RETRACTED));

        trg_shoot.and(trg_pass.negate()).onTrue(m_superstructure.activateOuttake(ShooterK.kShooterMaxRPS)).onFalse(m_superstructure.deactivateOuttake());
        trg_shoot.and(trg_pass).onTrue(m_superstructure.startPassing()).onFalse(m_superstructure.stopPassing());
        trg_emergencyBarf.onTrue(m_superstructure.activateOuttake(ShooterK.kShooterEmergencyRPS)).onFalse(m_superstructure.deactivateOuttake());

        // Override commands
        trg_maxShooterOverride.onTrue(m_superstructure.maxShooter()).onFalse(m_superstructure.stopShooter());

        trg_turret180Override.onTrue(m_superstructure.turretTo(180)).onFalse(m_superstructure.turretTo(0));

        trg_hood30Override.onTrue(m_superstructure.hoodTo(30)).onFalse(m_superstructure.hoodTo(0));

        trg_startSpindexerOverride.onTrue(m_superstructure.startSpindexer()).onFalse(m_superstructure.stopSpindexer());

        trg_startTunnelOverride.onTrue(m_superstructure.startTunnel()).onFalse(m_superstructure.stopTunnel());

        trg_maxRollersOverride.onTrue(m_superstructure.setIntakeRollersSpeed(IntakeRollersVelocity.MAX)).onFalse(m_superstructure.setIntakeRollersSpeed(IntakeRollersVelocity.STOP));

        trg_deployIntakeOverride.onTrue(m_superstructure.intakeTo(IntakeArmPosition.DEPLOYED)).onFalse(m_superstructure.intakeTo(IntakeArmPosition.SAFE));
        trg_intakeUpOverride.onTrue(m_superstructure.intakeTo(IntakeArmPosition.RETRACTED));
    }

    private void configureTestBindings() {
        // Intake
        // m_driver.a().onTrue(m_intake.setIntakeArmPos(IntakeArmPosition.RETRACTED));
        // m_driver.b().onTrue(m_intake.setIntakeArmPos(IntakeArmPosition.SAFE));
        // m_driver.x().onTrue(m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED));

        // m_driver.povRight().onTrue(m_intake.setIntakeRollersSpeed(IntakeIntakeRollersVelocity.MID));
        // m_driver.povDown().onTrue(m_intake.setIntakeRollersSpeed(IntakeIntakeRollersVelocity.STOP)); 
        // m_driver.povUp().onTrue(m_intake.setIntakeRollersSpeed(IntakeIntakeRollersVelocity.MAX));

        // Indexer
        // m_driver.povUp().onTrue(m_indexer.startSpindexer());
        // m_driver.povDown().onTrue(m_indexer.stopSpindexer());
        // m_driver.povLeft().onTrue(m_indexer.startTunnel());
        // m_driver.povRight().onTrue(m_indexer.stopTunnel());

        // Shooter
        // m_driver.povDown().onTrue(m_shooter.setShooterVelocityCmd(RotationsPerSecond.of(0)));
        // m_driver.povUp().onTrue(m_shooter.setShooterVelocityCmd(ShooterK.kShooterMaxRPS));

        // m_driver.a().onTrue(m_shooter.setHoodPositionCmd(ShooterK.kHoodMinDegs));
        // m_driver.y().onTrue(m_shooter.setHoodPositionCmd(ShooterK.kHoodMaxDegs));

        // m_driver.leftTrigger().onTrue(m_shooter.setTurretPositionCmd(ShooterK.kTurretMinRots));
        // m_driver.leftBumper().onTrue(m_shooter.setTurretPositionCmd(Rotations.of(0)));
        // m_driver.rightTrigger().onTrue(m_shooter.setTurretPositionCmd(ShooterK.kTurretMaxRots));

        // Test sequences
        trg_activateIntake.onTrue(m_superstructure.activateIntake());
        trg_prepIntake.onTrue(m_superstructure.deactivateIntake(IntakeArmPosition.SAFE));
        trg_retractIntake.onTrue(m_superstructure.deactivateIntake(IntakeArmPosition.RETRACTED));

        trg_shoot.and(trg_pass.negate()).onTrue(m_superstructure.activateOuttake(ShooterK.kShooterMaxRPS)).onFalse(m_superstructure.deactivateOuttake());
        trg_shoot.and(trg_pass).onTrue(m_superstructure.startPassing()).onFalse(m_superstructure.stopPassing());
        trg_emergencyBarf.onTrue(m_superstructure.activateOuttake(ShooterK.kShooterEmergencyRPS)).onFalse(m_superstructure.deactivateOuttake());

        // Override commands
        trg_maxShooterOverride.onTrue(m_superstructure.maxShooter());

        trg_turret180Override.onTrue(m_superstructure.turretTo(180));

        trg_hood30Override.onTrue(m_superstructure.hoodTo(30));

        trg_startSpindexerOverride.onTrue(m_superstructure.startSpindexer()).onFalse(m_superstructure.stopSpindexer());

        trg_startTunnelOverride.onTrue(m_superstructure.startTunnel()).onFalse(m_superstructure.stopTunnel());

        trg_maxRollersOverride.onTrue(m_superstructure.setIntakeRollersSpeed(IntakeRollersVelocity.MAX)).onFalse(m_superstructure.setIntakeRollersSpeed(IntakeRollersVelocity.STOP));

        trg_deployIntakeOverride.onTrue(m_superstructure.intakeTo(IntakeArmPosition.DEPLOYED)).onFalse(m_superstructure.intakeTo(IntakeArmPosition.SAFE));
        trg_intakeUpOverride.onTrue(m_superstructure.intakeTo(IntakeArmPosition.RETRACTED));
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        for (Vision camera : m_cameras) {
            Optional<EstimatedRobotPose> estimatedPoseOptional = camera.getEstimatedGlobalPose();
            if (estimatedPoseOptional.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
                Pose2d estimatedRobotPose2d = estimatedRobotPose.estimatedPose.toPose2d();
                var ctreTime = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
                m_drivetrain.addVisionMeasurement(estimatedRobotPose2d, ctreTime, camera.getEstimationStdDevs());                
                m_visionSeenLastSec = ctreTime;
            }
        }

        // periodics
        m_shooter.periodic();
        m_indexer.periodic();
        log_povUp.accept(m_driver.povUp());
        log_povDown.accept(m_driver.povDown());
        log_povLeft.accept(m_driver.povLeft());
        log_povRight.accept(m_driver.povRight());
        log_visionSeenPastSecond.accept((Utils.getCurrentTimeSeconds() - m_visionSeenLastSec) < 1.0);
    }

    @Override
    public void disabledInit() {
        m_waltAutonFactory.setAlliance(
            DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get().equals(Alliance.Red)
        );

        AutonChooser.initialize();

        m_autonList.putIfAbsent("oneRightNeutralPickup", m_waltAutonFactory.oneRightNeutralPickup());
        m_autonList.putIfAbsent("twoRightNeutralPickup", m_waltAutonFactory.twoRightNeutralPickup());
        m_autonList.putIfAbsent("threeRightNeutralPickup", m_waltAutonFactory.threeRightNeutralPickup());
        m_autonList.putIfAbsent("oneLeftNeutralPickup", m_waltAutonFactory.oneLeftNeutralPickup());
        m_autonList.putIfAbsent("twoLeftNeutralPickup", m_waltAutonFactory.twoLeftNeutralPickup());
        m_autonList.putIfAbsent("threeLeftNeutralPickup", m_waltAutonFactory.threeLeftNeutralPickup());

        m_autonomousCommand = m_autonList.get("threeRightNeutralPickup");
    }

    @Override
    public void disabledPeriodic() {
        if (!AutonChooser.m_chooser.getSelected().equals(m_autonChosen)) {
            switch (AutonChooser.m_chooser.getSelected()) {
                case "oneRightNeutralPickup":
                    AutonChooser.pub_autonName.set("One Right Neutral Pickup");
                    m_autonChosen = "oneRightNeutralPickup";
                    AutonChooser.pub_autonMade.set(false);
                    break;
                case "twoRightNeutralPickup":
                    AutonChooser.pub_autonName.set("Two Right Neutral Pickup");
                    m_autonChosen = "twoRightNeutralPickup";
                    AutonChooser.pub_autonMade.set(false);
                    break;
                case "threeRightNeutralPickup":
                    AutonChooser.pub_autonName.set("Three Right Neutral Pickup");
                    m_autonChosen = "threeRightNeutralPickup";
                    AutonChooser.pub_autonMade.set(false);
                    break;
                case "oneLeftNeutralPickup":
                    AutonChooser.pub_autonName.set("One Left Neutral Pickup");
                    m_autonChosen = "oneLeftNeutralPickup";
                    AutonChooser.pub_autonMade.set(false);
                    break;
                case "twoLeftNeutralPickup":
                    AutonChooser.pub_autonName.set("Two Left Neutral Pickup");
                    m_autonChosen = "twoLeftNeutralPickup";
                    AutonChooser.pub_autonMade.set(false);
                    break;
                case "threeLeftNeutralPickup":
                    AutonChooser.pub_autonName.set("Three Left Neutral Pickup");
                    m_autonChosen = "threeLeftNeutralPickup";
                    AutonChooser.pub_autonMade.set(false);
                    break;
            }
        }

        if (AutonChooser.sub_makeAuton.get()) {
            m_autonomousCommand = m_autonList.get(m_autonChosen);
            AutonChooser.pub_autonMade.set(true);
            AutonChooser.pub_makeAuton.set(false);
        }
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        SwerveDriveState robotState = m_drivetrain.getState();
        Pose2d robotPose = robotState.Pose;
        m_visionSim.simulationPeriodic(robotPose);
        m_drivetrain.simulationPeriodic();
        m_shooter.simulationPeriodic();
        m_intake.simulationPeriodic();
        m_indexer.simulationPeriodic();
    }
}
