// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IndexerK.kLogTab;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.VisionK;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.DeployPosition;
import frc.robot.subsystems.Intake.RollersVelocity;
import frc.robot.subsystems.Swerve;
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

    public final Swerve m_drivetrain = TunerConstants.createDrivetrain();
    private Command m_autonomousCommand;

    private final AutoFactory m_autoFactory = m_drivetrain.createAutoFactory();
    private final WaltAutonFactory m_waltAutonFactory = new WaltAutonFactory(m_autoFactory, m_drivetrain);

    private final VisionSim m_visionSim = new VisionSim();
    private final Vision m_camera1 = new Vision(VisionK.kCamera1CamName, VisionK.kCamera1CamSimVisualName, VisionK.kCamera1CamRoboToCam, m_visionSim, VisionK.kCamera1SimProps);
    private final Vision m_camera2 = new Vision(VisionK.kCamera2CamName, VisionK.kCamera2CamSimVisualName, VisionK.kCamera2CamRoboToCam, m_visionSim, VisionK.kCamera2SimProps);

    // this should be updated with all of our cameras
    private final Vision[] m_cameras = {m_camera1, m_camera2};

    private final Shooter m_shooter = new Shooter();
    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();

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

        m_driver.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
        m_driver.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driver.getLeftY(), -m_driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driver.back().and(m_driver.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_driver.back().and(m_driver.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_driver.start().and(m_driver.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driver.start().and(m_driver.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        m_driver.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        m_drivetrain.registerTelemetry(logger::telemeterize);

        /* CUSTOM BINDS */

        //robot heads toward fuel when detected :D (hypothetically)(robo could blow up instead)
        m_driver.x().whileTrue(m_drivetrain.swerveToObject());
    }

    private void configureTestBindings() {
        // Intake
        // m_driver.a().onTrue(m_intake.setDeployPos(DeployPosition.RETRACTED));
        // m_driver.b().onTrue(m_intake.setDeployPos(DeployPosition.SAFE));
        // m_driver.x().onTrue(m_intake.setDeployPos(DeployPosition.DEPLOYED));

        // m_driver.povRight().onTrue(m_intake.setRollersSpeed(RollersVelocity.MID));
        // m_driver.povDown().onTrue(m_intake.setRollersSpeed(RollersVelocity.STOP)); 
        // m_driver.povUp().onTrue(m_intake.setRollersSpeed(RollersVelocity.MAX));

        // Indexer
        // m_driver.povUp().onTrue(m_indexer.startSpinner());
        // m_driver.povDown().onTrue(m_indexer.stopSpinner());
        // m_driver.povLeft().onTrue(m_indexer.startExhaust());
        // m_driver.povRight().onTrue(m_indexer.stopExhaust());

        // Shooter
        m_driver.povDown().onTrue(m_shooter.setFlywheelVelocityCmd(RotationsPerSecond.of(0)));
        m_driver.povUp().onTrue(m_shooter.setFlywheelVelocityCmd(ShooterK.kFlywheelMaxRPS));

        m_driver.a().onTrue(m_shooter.setHoodPositionCmd(ShooterK.kHoodMinDegs));
        m_driver.y().onTrue(m_shooter.setHoodPositionCmd(ShooterK.kHoodMaxDegs));

        m_driver.leftTrigger().onTrue(m_shooter.setTurretPositionCmd(ShooterK.kTurretMinRots));
        m_driver.leftBumper().onTrue(m_shooter.setTurretPositionCmd(Rotations.of(0)));
        m_driver.rightTrigger().onTrue(m_shooter.setTurretPositionCmd(ShooterK.kTurretMaxRots));
    }

    public Command getAutonomousCommand() {
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
            }
        }

        // periodics
        m_shooter.periodic();
        m_indexer.periodic();
        log_povUp.accept(m_driver.povUp());
        log_povDown.accept(m_driver.povDown());
        log_povLeft.accept(m_driver.povLeft());
        log_povRight.accept(m_driver.povRight());
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
