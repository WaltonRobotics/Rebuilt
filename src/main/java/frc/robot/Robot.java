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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Autons.AutonChooser;
import frc.robot.Autons.WaltAutonFactory;
import frc.robot.Constants.VisionK;
import frc.robot.generated.TunerConstants;
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

    private final CommandXboxController driver = new CommandXboxController(0);

    public final Swerve drivetrain = TunerConstants.createDrivetrain();
    private Command m_autonomousCommand;

    private boolean autonMade = false;
    private Command desiredAuton;

    private final VisionSim visionSim = new VisionSim();
    private final Vision camera1 = new Vision(VisionK.kCamera1CamName, VisionK.kCamera1CamSimVisualName, VisionK.kCamera1CamRoboToCam, visionSim, VisionK.kCamera1SimProps);
    private final Vision camera2 = new Vision(VisionK.kCamera2CamName, VisionK.kCamera2CamSimVisualName, VisionK.kCamera2CamRoboToCam, visionSim, VisionK.kCamera2SimProps);

    private final Detection detection = new Detection();

    // this should be updated with all of our cameras
    private final Vision[] cameras = {camera1, camera2};

    private final AutoFactory autoFactory = drivetrain.createAutoFactory();
    private final WaltAutonFactory waltAutonFactory = new WaltAutonFactory(autoFactory, drivetrain);

    private final Intake intake = new Intake();
  
    private final Indexer m_indexer = new Indexer();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        configureBindings();
    }

    private Command driveCommand() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will execute this command periodically
        return drivetrain.applyRequest(() -> {
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

        //robot heads toward fuel when detected :D (hypothetically)(robo could blow up instead)
        driver.x().whileTrue(drivetrain.swerveToObject());

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.a().onTrue(intake.setDeployPos(DeployPosition.RETRACTED));
        driver.b().onTrue(intake.setDeployPos(DeployPosition.SAFE));
        driver.x().onTrue(intake.setDeployPos(DeployPosition.DEPLOYED));

        driver.povRight().onTrue(intake.setRollersSpeed(RollersVelocity.MID));
        driver.povDown().onTrue(intake.setRollersSpeed(RollersVelocity.STOP)); 
        driver.povUp().onTrue(intake.setRollersSpeed(RollersVelocity.MAX));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* CUSTOM BINDS */
        driver.povUp().onTrue(m_indexer.startSpinner());
        driver.povDown().onTrue(m_indexer.stopSpinner());
        driver.povLeft().onTrue(m_indexer.startExhaust());
        driver.povRight().onTrue(m_indexer.stopExhaust());
    }

    public Command getAutonomousCommand(Command auton) {
        return auton;
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        for (Vision camera : cameras) {
            Optional<EstimatedRobotPose> estimatedPoseOptional = camera.getEstimatedGlobalPose();
            if (estimatedPoseOptional.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
                Pose2d estimatedRobotPose2d = estimatedRobotPose.estimatedPose.toPose2d();
                var ctreTime = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
                drivetrain.addVisionMeasurement(estimatedRobotPose2d, ctreTime, camera.getEstimationStdDevs());
            }
        }

        // Periodics
        m_indexer.periodic();
        log_povUp.accept(driver.povUp());
        log_povDown.accept(driver.povDown());
        log_povLeft.accept(driver.povLeft());
        log_povRight.accept(driver.povRight());
    }

    @Override
    public void disabledInit() {
         waltAutonFactory.setAlliance( 
            DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get().equals(Alliance.Red)
        );

        AutonChooser.initialize();
    }

    @Override
    public void disabledPeriodic() {
        waltAutonFactory.setAlliance( 
            DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get().equals(Alliance.Red)
        );

        if (!autonMade) {
            if (AutonChooser.m_chooser.getSelected().equals("oneNeutralPickup")) {
                desiredAuton = waltAutonFactory.oneNeutralPickup();
                AutonChooser.pub_autonName.set("One Neutral Pickup");
                autonMade = true;
                AutonChooser.pub_autonMade.set(true);
            }   

            if (AutonChooser.m_chooser.getSelected().equals("twoNeutralPickup")) {
                desiredAuton = waltAutonFactory.twoNeutralPickup();
                AutonChooser.pub_autonName.set("Two Neutral Pickup");
                autonMade = true;
                AutonChooser.pub_autonMade.set(true);
            }

            if (AutonChooser.m_chooser.getSelected().equals("threeNeutralPickup")) {
                desiredAuton = waltAutonFactory.threeNeutralPickup();
                AutonChooser.pub_autonName.set("Three Neutral Pickup");
                autonMade = true;
                AutonChooser.pub_autonMade.set(true);
            }
        }
        if (AutonChooser.sub_refreshChoice.getAsBoolean()) {
            AutonChooser.pub_refreshChoice.set(false);
            AutonChooser.pub_autonName.set("No Auton Made");

            autonMade = false;
            AutonChooser.pub_autonMade.set(false);
        }
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = getAutonomousCommand(desiredAuton);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
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
        SwerveDriveState robotState = drivetrain.getState();
        Pose2d robotPose = robotState.Pose;
        visionSim.simulationPeriodic(robotPose);
        drivetrain.simulationPeriodic();
        intake.simulationPeriodic();
        m_indexer.simulationPeriodic();
    }
}
