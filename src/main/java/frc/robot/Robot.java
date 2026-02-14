// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IndexerK.kLogTab;
import static frc.robot.Constants.ShooterK.kFlywheelLowRPS;
import static frc.robot.Constants.ShooterK.kFlywheelMaxRPS;
import static frc.robot.Constants.ShooterK.kFlywheelZeroRPS;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.VisionK;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.DeployPosition;
import frc.robot.subsystems.Intake.RollersVelocity;
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
    private final BooleanLogger log_visionSeenPastSecond = new BooleanLogger("Robot", "VisionSeenLastSec");

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController manipulator = new CommandXboxController(1);

    public final Swerve m_drivetrain = TunerConstants.createDrivetrain();
    private Command m_autonomousCommand;

    private final AutoFactory m_autoFactory = m_drivetrain.createAutoFactory();
    private final WaltAutonFactory m_waltAutonFactory = new WaltAutonFactory(m_autoFactory, m_drivetrain);

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

    private Trigger trg_driverOverride = driver.b();
    private Trigger trg_manipOverride = manipulator.b();
    private Trigger trg_inOverride = trg_driverOverride.or(trg_manipOverride);

    // Command sequence triggers
    private Trigger trg_activateIntake = manipulator.a().and(trg_manipOverride.negate());
    private Trigger trg_prepIntake = manipulator.x().and(trg_manipOverride.negate());
    private Trigger trg_retractIntake = manipulator.y().and(trg_manipOverride.negate());

    private Trigger trg_normalOuttake = driver.rightTrigger().and(trg_manipOverride.negate());
    private Trigger trg_emergencyOuttake = driver.leftTrigger().and(trg_manipOverride.negate());

    private Trigger trg_startPassing = manipulator.rightBumper().and(trg_manipOverride.negate());
    private Trigger trg_stopPassing = manipulator.leftBumper().and(trg_manipOverride.negate());

    // Override triggers
    private Trigger trg_maxShooter = trg_manipOverride.and(manipulator.x());
    private Trigger trg_stopShooter = trg_manipOverride.and(manipulator.y());

    private Trigger trg_turret180 = trg_manipOverride.and(manipulator.povRight());
    private Trigger trg_turret0 = trg_manipOverride.and(manipulator.povLeft());

    private Trigger trg_hood30 = trg_manipOverride.and(manipulator.povUp());
    private Trigger trg_hood0 = trg_manipOverride.and(manipulator.povDown());

    private Trigger trg_startSpinner = trg_manipOverride.and(manipulator.rightBumper());

    private Trigger trg_startExhaust = trg_manipOverride.and(manipulator.leftBumper());

    private Trigger trg_maxRollers = trg_manipOverride.and(manipulator.a());

    private Trigger trg_deployIntake = trg_manipOverride.and(manipulator.rightTrigger());
    private Trigger trg_intakeUp = trg_manipOverride.and(manipulator.leftTrigger());

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        // configureBindings();
        configureTestBindings();    //this should be commented out during competition matches
    }

    private Command driveCommand() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will execute this command periodically
        return m_drivetrain.applyRequest(() -> {
            var angularRate = driver.leftTrigger().getAsBoolean() ? 
            kMaxHighAngularRate : kMaxAngularRate;
        
            var driverXVelo = -driver.getLeftY() * kMaxTranslationSpeed;
            var driverYVelo = -driver.getLeftX() * kMaxTranslationSpeed;
            var driverYawRate = -driver.getRightX() * angularRate;

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
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        m_drivetrain.registerTelemetry(logger::telemeterize);

        /* CUSTOM BINDS */

        //robot heads toward fuel when detected :D (hypothetically)(robo could blow up instead)
        driver.x().whileTrue(m_drivetrain.swerveToObject());
    }

    private void configureTestBindings() {
        // Intake
        // driver.a().onTrue(m_intake.setDeployPos(DeployPosition.RETRACTED));
        // driver.b().onTrue(m_intake.setDeployPos(DeployPosition.SAFE));
        // driver.x().onTrue(m_intake.setDeployPos(DeployPosition.DEPLOYED));

        // driver.povRight().onTrue(m_intake.setRollersSpeed(RollersVelocity.MID));
        // driver.povDown().onTrue(m_intake.setRollersSpeed(RollersVelocity.STOP)); 
        // driver.povUp().onTrue(m_intake.setRollersSpeed(RollersVelocity.MAX));

        // Indexer
        // driver.povUp().onTrue(m_indexer.startSpinner());
        // driver.povDown().onTrue(m_indexer.stopSpinner());
        // driver.povLeft().onTrue(m_indexer.startExhaust());
        // driver.povRight().onTrue(m_indexer.stopExhaust());

        // Shooter
        // driver.povDown().onTrue(m_shooter.setFlywheelVelocityCmd(RotationsPerSecond.of(0)));
        // driver.povUp().onTrue(m_shooter.setFlywheelVelocityCmd(ShooterK.kFlywheelMaxRPS));

        // driver.a().onTrue(m_shooter.setHoodPositionCmd(ShooterK.kHoodMinDegs));
        // driver.y().onTrue(m_shooter.setHoodPositionCmd(ShooterK.kHoodMaxDegs));

        // driver.leftTrigger().onTrue(m_shooter.setTurretPositionCmd(ShooterK.kTurretMinRots));
        // driver.leftBumper().onTrue(m_shooter.setTurretPositionCmd(Rotations.of(0)));
        // driver.rightTrigger().onTrue(m_shooter.setTurretPositionCmd(ShooterK.kTurretMaxRots));

        // Test sequences
        trg_activateIntake.onTrue(m_superstructure.activateIntake());
        trg_prepIntake.onTrue(m_superstructure.prepIntake());
        trg_retractIntake.onTrue(m_superstructure.retractIntake());

        trg_normalOuttake.onTrue(m_superstructure.normalOuttake()).onFalse(m_superstructure.deactivateOuttake());
        trg_emergencyOuttake.onTrue(m_superstructure.emergencyOuttake()).onFalse(m_superstructure.deactivateOuttake());

        trg_startPassing.onTrue(m_superstructure.startPassing());
        trg_stopPassing.onTrue(m_superstructure.stopPassing());

        // Override commands
        trg_maxShooter.onTrue(m_shooter.setFlywheelVelocityCmd(kFlywheelMaxRPS));
        trg_stopShooter.onTrue(m_shooter.setFlywheelVelocityCmd(kFlywheelZeroRPS));

        trg_turret180.onTrue(m_shooter.setTurretPositionCmd(Angle.ofBaseUnits(180, Degree)));
        trg_turret0.onTrue(m_shooter.setTurretPositionCmd(Angle.ofBaseUnits(0, Degree)));

        trg_hood30.onTrue(m_shooter.setHoodPositionCmd(Angle.ofBaseUnits(30, Degree)));
        trg_hood0.onTrue(m_shooter.setHoodPositionCmd(Angle.ofBaseUnits(0, Degree)));

        trg_startSpinner.onTrue(m_indexer.startSpinner()).onFalse(m_indexer.stopSpinner());

        trg_startExhaust.onTrue(m_indexer.startExhaust()).onFalse(m_indexer.stopExhaust());

        trg_maxRollers.onTrue(m_intake.setRollersSpeed(RollersVelocity.MAX)).onFalse(m_intake.setRollersSpeed(RollersVelocity.STOP));

        trg_deployIntake.onTrue(m_intake.setDeployPos(DeployPosition.DEPLOYED)).onFalse(m_intake.setDeployPos(DeployPosition.SAFE));
        trg_intakeUp.onTrue(m_intake.setDeployPos(DeployPosition.RETRACTED));
    }

    public Command getAutonomousCommand() {
        m_waltAutonFactory.setAlliance( 
            DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get().equals(Alliance.Red)
        );

        return m_waltAutonFactory.threeNeutralPickup();
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
        log_povUp.accept(driver.povUp());
        log_povDown.accept(driver.povDown());
        log_povLeft.accept(driver.povLeft());
        log_povRight.accept(driver.povRight());
        log_visionSeenPastSecond.accept((Utils.getCurrentTimeSeconds() - m_visionSeenLastSec) < 1.0);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = getAutonomousCommand();

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
