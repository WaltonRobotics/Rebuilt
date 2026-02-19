// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.VisionK;
import static frc.robot.Constants.RobotK.*;
import frc.robot.subsystems.Shooter.TurretGoal;
import frc.robot.subsystems.Shooter.TurretPosition;
import frc.robot.subsystems.shooter.FuelSim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Robot extends TimedRobot {
    private final double kMaxTranslationSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final double kMaxHighAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    private final DoubleLogger log_stickDesiredFieldX = WaltLogger.logDouble("Swerve", "stick desired teleop x");
    private final DoubleLogger log_stickDesiredFieldY = WaltLogger.logDouble("Swerve", "stick desired teleop y");
    private final DoubleLogger log_stickDesiredFieldZRot = WaltLogger.logDouble("Swerve", "stick desired teleop z rot");

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

    public final Swerve m_drivetrain = TunerConstants.createDrivetrain();
    private Command m_autonomousCommand;
    private final AutoFactory autoFactory = m_drivetrain.createAutoFactory();


    private final VisionSim visionSim = new VisionSim();
    private final Vision camera1 = new Vision(VisionK.kCamera1CamName, VisionK.kCamera1CamSimVisualName, VisionK.kCamera1CamRoboToCam, visionSim, VisionK.kCamera1SimProps);
    private final Vision camera2 = new Vision(VisionK.kCamera2CamName, VisionK.kCamera2CamSimVisualName, VisionK.kCamera2CamRoboToCam, visionSim, VisionK.kCamera2SimProps);

    // this should be updated with all of our cameras
    private final Vision[] cameras = {camera1, camera2};

    private final Shooter m_shooter;


    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_shooter = new Shooter(() -> m_drivetrain.getState().Pose, () -> m_drivetrain.getChassisSpeeds());

        // m_shooter.setDefaultCommand(m_shooter.shooterDefaultCommands());
        m_shooter.zeroHoodCmd();
        m_shooter.zeroTurretCmd();
        configureBindings();

        if(Robot.isSimulation()) {
            configureFuelSim();
            FuelSim.getInstance().enableAirResistance();
        }
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

    //(nonsotm (just for simulating entire robot)) BLARGHHHHHH get intake dude (alex?) to give me his code (idk if he finished it yet)
    private void configureFuelSim() {
        FuelSim instance = FuelSim.getInstance();
        // instance.spawnStartingFuel();
        

        instance.registerRobot(
                kRobotFullWidth.in(Meters),
                kRobotFullLength.in(Meters),
                kBumperHeight.in(Meters),
                () -> m_drivetrain.getState().Pose,
                () -> m_drivetrain.getChassisSpeeds());
        // instance.registerIntake(
        //         -kRobotFullLength.div(2).in(Meters),
        //         kRobotFullLength.div(2).in(Meters),
        //         -kRobotFullWidth.div(2).plus(Inches.of(7)).in(Meters),
        //         -kRobotFullWidth.div(2).in(Meters),
        //         () -> intake.isRightDeployed() && m_shooter.simAbleToIntake(),
        //         m_shooter::simIntake);
        // instance.registerIntake(
        //         -kRobotFullLength.div(2).in(Meters),
        //         kRobotFullLength.div(2).in(Meters),
        //         kRobotFullWidth.div(2).in(Meters),
        //         kRobotFullWidth.div(2).plus(Inches.of(7)).in(Meters),
        //         () -> intake.isLeftDeployed() && m_shooter.simAbleToIntake(),
        //         m_shooter::simIntake);

        instance.start();
        instance.logFuels();
        SmartDashboard.putData(Commands.runOnce(() -> {
                    FuelSim.getInstance().clearFuel();
                    FuelSim.getInstance().spawnStartingFuel();
                })
                .withName("Reset Fuel")
                .ignoringDisable(true));
    }

    private void configureBindings() {
        /* SWERVE BINDS */
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

        // driver.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
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
        // driver.povUp().onTrue(m_shooter.setFlywheelVelocityCmd(FlywheelVelocity.ZERO));
        // driver.povRight().onTrue(m_shooter.setFlywheelVelocityCmd(FlywheelVelocity.SCORE));
        // driver.povDown().onTrue(m_shooter.setFlywheelVelocityCmd(FlywheelVelocity.MAX));
        // driver.povLeft().onTrue(m_shooter.setFlywheelVelocityCmd(FlywheelVelocity.PASS));

        // driver.leftBumper().onTrue(m_shooter.setHoodPositionCmd(HoodPosition.MIN));
        // driver.rightBumper().onTrue(m_shooter.setHoodPositionCmd(HoodPosition.SCORE));
        // driver.leftTrigger().onTrue(m_shooter.setHoodPositionCmd(HoodPosition.PASS));
        // driver.y().onTrue(m_shooter.setHoodPositionCmd(HoodPosition.MAX));

        // driver.leftBumper().onTrue(m_shooter.setTurretPositionCmd(TurretPosition.SCORE));
        // driver.rightBumper().onTrue(shooter.setTurretPositionCmd(TurretPosition.PASS));
        // driver.leftTrigger().onTrue(m_shooter.setTurretPositionCmd(TurretPosition.MIN));
        driver.leftTrigger().onTrue(m_shooter.setTurretPositionCmd(TurretPosition.MAX));

        driver.rightTrigger().onTrue(m_shooter.zeroShooterCmd());

        driver.a().whileTrue(Commands.repeatingSequence(
            Commands.runOnce(m_shooter::launchFuel),
            Commands.waitSeconds(0.1)));  
        driver.b().onTrue(Commands.runOnce(() -> {
            FuelSim.getInstance().clearFuel();
            // FuelSim.getInstance().spawnStartingFuel();
        }));

        driver.x().onTrue(m_shooter.setGoal(TurretGoal.SCORING));
        driver.leftBumper().onTrue(m_shooter.setGoal(TurretGoal.OFF));
        driver.y().onTrue(m_shooter.setGoal(TurretGoal.TEST));
        }

    public Command getAutonomousCommand() {
        // Simple Auton (hardcoded)
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            autoFactory.resetOdometry("testAlexandra"),
            autoFactory.trajectoryCmd("testAlexandra")
        );
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
                m_drivetrain.addVisionMeasurement(estimatedRobotPose2d, ctreTime, camera.getEstimationStdDevs());
            }
        }
        // periodics
        m_shooter.periodic();
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
        m_shooter.zeroShooterCmd();
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
    public void simulationInit() {
        FuelSim.getInstance().start();
    }

    @Override
    public void simulationPeriodic() {
        FuelSim instance = FuelSim.getInstance();
        instance.logFuels();
        instance.updateSim();
        SwerveDriveState robotState = m_drivetrain.getState();
        Pose2d robotPose = robotState.Pose;
        visionSim.simulationPeriodic(robotPose);
        // RobotState.getInstance().resetPose(robotPose);
        m_drivetrain.simulationPeriodic();
        m_shooter.simulationPeriodic();
    }
}
