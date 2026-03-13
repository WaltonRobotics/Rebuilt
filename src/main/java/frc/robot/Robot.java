
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.RobotK.*;
import static frc.robot.Constants.ShooterK.kShooterRPS;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.shooter.FuelSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.Constants.RobotK;
import frc.robot.dashboards.AutonChooser;
import frc.robot.dashboards.TestingDashboard;
import frc.robot.autons.WaltAutonFactory;
import frc.robot.autons.WaltSimpleAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.Indexer;
import frc.robot.vision.WaltCamera;
import frc.util.Telemetry;
// import frc.util.WaltVisualSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;

public class Robot extends TimedRobot {
    /* CLASS VARIABLES */
    //---CONSTANTS
    private final LinearVelocity kMaxTranslationSpeed = TunerConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    private final AngularVelocity kDriverMaxAngularRate = RotationsPerSecond.of(1.05); // 3/4 of a rotation per second max angular velocity
    private final AngularVelocity kSwerveShimmyAngularRate = RotationsPerSecond.of(1.3 / 2);

    private double m_visionSeenLastSec = Utils.getCurrentTimeSeconds();
    private final BooleanLogger log_visionSeenPastSecond = new BooleanLogger(kLogTab, "VisionSeenLastSec");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxTranslationSpeed.times(0.1)).withRotationalDeadband(kDriverMaxAngularRate.times(0.1)) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(kMaxTranslationSpeed.in(MetersPerSecond));

    //---CONTROLLERS
    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_manipulator = new CommandXboxController(1);

    //---INIT SUBSYSTEMS
    public final Swerve m_drivetrain = TunerConstants.createDrivetrain();

    private final Shooter m_shooter = new Shooter(() -> m_drivetrain.getState().Pose, () -> m_drivetrain.getChassisSpeeds());

    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();

    // private final WaltVisualSim m_visualSim;
    private final Superstructure m_superstructure = new Superstructure(m_intake, m_indexer, m_shooter);

    //---AUTONS
    private Command m_autonomousCommand;

    private final AutoFactory m_autoFactory = m_drivetrain.createAutoFactory();
    private final WaltAutonFactory m_waltAutonFactory = new WaltAutonFactory(m_autoFactory, m_drivetrain);

    private final WaltSimpleAutonFactory m_simpleAutonFactory = new WaltSimpleAutonFactory(m_superstructure, m_autoFactory, m_intake, m_shooter, m_drivetrain);
    //---VISION

    private PowerDistribution m_PDH = new PowerDistribution();
    // private final VisionSim m_visionSim = new VisionSim();

    /* TRIGGERS */
    private Trigger trg_driverOverride = m_driver.b();
    private Trigger trg_manipOverride = m_manipulator.b();

    //---COMMAND SEQUENCE TRIGGERS

    // private Trigger trg_activateIntake = m_manipulator.a().and(trg_manipOverride.negate());
    // private Trigger trg_safeIntake = m_manipulator.x().and(trg_manipOverride.negate());
    private Trigger trg_intake = m_manipulator.rightTrigger().and(trg_manipOverride.negate());
    private Trigger trg_retractIntake = m_manipulator.rightBumper().and(trg_manipOverride.negate());

    private Trigger trg_shoot = m_driver.rightTrigger().and(trg_driverOverride.negate());
    private Trigger trg_emergencyBarf = m_driver.rightTrigger().and(trg_driverOverride);

    private Trigger trg_passLeft = m_manipulator.povLeft().and(trg_driverOverride.negate());
    private Trigger trg_passRight = m_manipulator.povRight().and(trg_driverOverride.negate()).and(trg_passLeft.negate());

    private Trigger trg_shimmy = m_manipulator.leftBumper();
    private Trigger trg_swerveShimmy = m_driver.leftBumper();

    //---OVERRIDE TRIGGERS
    private Trigger trg_deployIntakeOverride = trg_manipOverride.and(m_manipulator.rightTrigger());
    private Trigger trg_intakeUpOverride = trg_manipOverride.and(m_manipulator.leftTrigger());

    private final Trigger trg_limitFPS = RobotModeTriggers.disabled();
    private final Trigger trg_unlimitFps = RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop());

    private Trigger trg_unjam = m_driver.rightBumper();


    /* LOGGERS */
    private final DoubleLogger log_stickDesiredFieldX = WaltLogger.logDouble("Swerve", "stick desired teleop x");
    private final DoubleLogger log_stickDesiredFieldY = WaltLogger.logDouble("Swerve", "stick desired teleop y");
    private final DoubleLogger log_stickDesiredFieldZRot = WaltLogger.logDouble("Swerve", "stick desired teleop z rot");
    private final DoubleLogger log_robotDesiredFieldZRot = WaltLogger.logDouble("Swerve", "robot desired teleop z rot");
    private final BooleanLogger log_swerveShimmying = WaltLogger.logBoolean("Swerve", "swerveShimmying");
    private final BooleanLogger log_povUp = WaltLogger.logBoolean(kLogTab, "Pov Up");
    private final BooleanLogger log_povRight = WaltLogger.logBoolean(kLogTab, "Pov Right");
    private final BooleanLogger log_povLeft = WaltLogger.logBoolean(kLogTab, "Pov Left");
    private final BooleanLogger log_povDown = WaltLogger.logBoolean(kLogTab, "Pov Down");

    private final BooleanLogger log_manipLeftTrigger = WaltLogger.logBoolean(kLogTab, "Manip Left Trigger");
    private final BooleanLogger log_manipRightTrigger = WaltLogger.logBoolean(kLogTab, "Manip Right Trigger");
    private final BooleanLogger log_driverLeftTrigger = WaltLogger.logBoolean(kLogTab, "Driver Left Trigger");
    private final BooleanLogger log_driverRightTrigger = WaltLogger.logBoolean(kLogTab, "Driver Right Trigger");

    private final DoubleLogger log_miniPCCurrent = WaltLogger.logDouble(kLogTab, "MiniPC current");

    private final BooleanLogger log_isDisabled = WaltLogger.logBoolean(kLogTab, "is robot disabled");

    private final Tracer m_periodicTracer = new Tracer();

    /* CONSTRUCTOR */
    public Robot() {
        configureBindings();
        // configureTestBindings();    //this should be commented out during competition matches
        // configureTestingDashboard();

        lastGotTagMsmtTimer.start();
        if (Robot.isSimulation()) {
            configureFuelSim();
            // FuelSim.getInstance().enableAirResistance();
        }
        // set FPS limit on boot
        WaltCamera.setFpsLimit(true);

        DriverStation.silenceJoystickConnectionWarning(true);
        PhotonCamera.setVersionCheckEnabled(false);
        LiveWindow.disableAllTelemetry();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        addPeriodic(m_shooter::fastPeriodic, 0.0025);
    }

    /* COMMANDS */
    /**
     * 
     * @param speedMultiplier How much you want to limit speed as a decimal percentage of kMaxTranslation. 1 does nothing
     * @return swerve drive command
     */
    private Command driveCommand(double speedMultiplier) {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will execute this command periodically
        return m_drivetrain.applyRequest(() -> {
            LinearVelocity translationSpeed = (m_driver.leftTrigger().getAsBoolean() ? 
                kMaxTranslationSpeed.times(speedMultiplier) :
                kMaxTranslationSpeed);
            
            var driverXVelo = translationSpeed.times(-m_driver.getLeftY());
            var driverYVelo = translationSpeed.times(-m_driver.getLeftX());
            var driverYawRate = kDriverMaxAngularRate.times(-m_driver.getRightX());

            log_stickDesiredFieldX.accept(driverXVelo.in(MetersPerSecond));
            log_stickDesiredFieldY.accept(driverYVelo.in(MetersPerSecond));
            log_stickDesiredFieldZRot.accept(driverYawRate.in(RotationsPerSecond));
            log_robotDesiredFieldZRot.accept(driverYawRate.in(RotationsPerSecond)); // Should be equal to stick desired IN THIS CASE
            log_swerveShimmying.accept(false);
                
            return drive
                .withVelocityX(driverXVelo) // Drive forward with Y (forward)
                .withVelocityY(driverYVelo) // Drive left with X (left)
                .withRotationalRate(driverYawRate); // Drive counterclockwise with negative X (left)
            }
        );
    }
    /**
     * Returns a modified swerveRequest which uses swerveShimmy's rotational value instead of the driver's stick's.
     * Does not move the robot, only returns a swerveRequest.
     * @param speedMultiplier How much you want to limit speed as a decimal percentage of kMaxTranslation. 1 does nothing
     * @param shimmyCCW If the shimmy direction is CCW
     * @return A swerveRequest with shimmying rotational values
     */
    private Command swerveRequestWithSwerveShimmy(double speedMultiplier, boolean shimmyCCW) {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will NOT execute this command periodically
        return m_drivetrain.applyRequest(() -> {
            LinearVelocity translationSpeed = (m_driver.leftTrigger().getAsBoolean() ? 
                kMaxTranslationSpeed.times(speedMultiplier) :
                kMaxTranslationSpeed);
            
            var driverXVelo = translationSpeed.times(-m_driver.getLeftY());
            var driverYVelo = translationSpeed.times(-m_driver.getLeftX());
            // Use shimmyAngularRate over driver request
            var yawRate = kSwerveShimmyAngularRate.times(shimmyCCW ? 1 : -1);

            log_stickDesiredFieldX.accept(driverXVelo.in(MetersPerSecond));
            log_stickDesiredFieldY.accept(driverYVelo.in(MetersPerSecond));
            log_stickDesiredFieldZRot.accept(kDriverMaxAngularRate.times(-m_driver.getRightX()).in(RotationsPerSecond));  // Logging driver requests even though they have no impact
            log_robotDesiredFieldZRot.accept(yawRate.in(RotationsPerSecond));
            log_swerveShimmying.accept(true);
                
            return drive
                .withVelocityX(driverXVelo) // Drive forward with Y (forward)
                .withVelocityY(driverYVelo) // Drive left with X (left)
                .withRotationalRate(yawRate); // Use shimmy rotation rate
            }
        );
    }
    /**
     * A modified driveCommand that utilizes the swerveShimmy rotational value instead of what the driver requests.
     * Translational control should not be affected. Regular driveCommand is used as defaultCommand.
     * @param speedMultiplier How much you want to limit speed as a decimal percentage of kMaxTranslation. 1 does nothing
     * @return Swerve driveCommand with swerveShimmy rotational values
     */
    private Command driveCommandWithSwerveShimmying(double speedMultiplier) {
        // Return a sequence of two swerveRequests to shimmy in alternating directions
        return Commands.repeatingSequence(
            swerveRequestWithSwerveShimmy(speedMultiplier, true).withTimeout(0.113),
            swerveRequestWithSwerveShimmy(speedMultiplier, false).withTimeout(0.1)
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

    //---BINDINGS
    private void configureBindings() {
        /* GENERATED SWERVE BINDS */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(driveCommand(RobotK.kRobotSpeedIntakingLimit));

        // Trigger trg_hubActiveOrPassing =
        //     new Trigger(
        //         () ->
        //             HubShiftUtil.getShiftedShiftInfo().active()
        //                 || m_shooter.getCurrentGoal().equals(ShooterGoal.PASSING));

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

        // Reset the field-centric heading on left bumper press.
        m_driver.leftBumper().and(trg_driverOverride).onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        m_drivetrain.registerTelemetry(logger::telemeterize);

        /* CUSTOM BINDS */
        trg_limitFPS.onTrue(WaltCamera.setFpsLimitCmd(true));   
        trg_unlimitFps.onTrue(WaltCamera.setFpsLimitCmd(false));

        //robot heads toward fuel when detected :D (hypothetically)(robo could blow up instead)
        // trg_swerveToObject.whileTrue(
        //     m_drivetrain.swerveToObject()
        // );

        //---NORMAL SEQUENCES
        //Intake
        trg_intake.and((trg_passLeft.or(trg_passRight)).negate()).whileTrue(
            m_superstructure.intake(() -> false)
        );
        trg_retractIntake.onTrue(
            m_superstructure.deactivateIntake(IntakeArmPosition.RETRACTED)
        );

        //Shooting
        // trg_shoot.and(() -> trg_hubActiveOrPassing.getAsBoolean()).whileTrue(
        trg_shoot.whileTrue(
            m_superstructure.activateOuttake(kShooterRPS)
        );

        // snapshot on each shoot press
        trg_shoot.onTrue(WaltCamera.takeSnapshotCmd());

        trg_intake.and(trg_passLeft.or(trg_passRight)).whileTrue(
            m_superstructure.intake(() -> true)
        );

        trg_emergencyBarf.whileTrue(
            m_superstructure.emergencyBarf()
        );
        
        trg_shimmy.whileTrue(m_superstructure.intakeArmShimmy());

        trg_swerveShimmy.whileTrue(driveCommandWithSwerveShimmying(RobotK.kRobotSpeedIntakingLimit));

        trg_swerveShimmy.onFalse(Commands.print("Robot swerveShimmy stopped"));

        trg_unjam.whileTrue(
            m_superstructure.unjamCmd()
        );

        //---OVERRIDE COMMANDS
        m_manipulator.x().and(trg_manipOverride).onTrue(m_intake.intakeArmCurrentSenseHoming());

        // m_manipulator.y().and(trg_manipOverride).onTrue(m_shooter.setHoodPositionCmd(Degrees.of(35)));
        // m_manipulator.a().and(trg_manipOverride).onTrue(m_shooter.setHoodPositionCmd(Degrees.of(1)));

        trg_deployIntakeOverride.onTrue(
            m_superstructure.intakeTo(IntakeArmPosition.DEPLOYED)
        ).onFalse(
            m_superstructure.intakeTo(IntakeArmPosition.SAFE)
        );
        trg_intakeUpOverride.onTrue(
            m_superstructure.intakeTo(IntakeArmPosition.RETRACTED)
        );

        // m_driver.y().and(trg_driverOverride).onTrue(m_shooter.turretHomingCmd(false));  //false? im not sure

        m_driver.povDown().onTrue(Commands.runOnce(() -> m_shooter.setShotCalcCmd(false)));
        m_driver.povRight().onTrue(Commands.runOnce(() -> m_shooter.setShotCalcCmd(true)));
    }

    private void configureTestBindings() {
        /* GENERATED SWERVE BINDS */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(driveCommand(RobotK.kRobotSpeedIntakingLimit));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        m_driver.leftBumper().and(trg_manipOverride.negate()).onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        m_drivetrain.registerTelemetry(logger::telemeterize);

        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driver.back().and(m_driver.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        // m_driver.back().and(m_driver.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driver.start().and(m_driver.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driver.start().and(m_driver.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    private void configureTestingDashboard() {
        /* INITIALIZE DASHBOARD */
        TestingDashboard.initialize();

        /* ELASTIC WIDGET BINDINGS */
        TestingDashboard.trg_letShooterVelocityRPSChange
            .whileTrue(m_shooter.setShooterVelocityCmd(TestingDashboard.sub_shooterVelocityRPS));
        TestingDashboard.trg_letTurretPositionRotsChange
            .whileTrue(m_shooter.setTurretPositionCmd(TestingDashboard.sub_turretPositionRots));
        // TestingDashboard.trg_letHoodPositionDegsChange
            // .whileTrue(m_shooter.setHoodPositionCmd(TestingDashboard.sub_hoodPositionDegs));

        TestingDashboard.trg_letSpindexerVelocityRPSChange
            .whileTrue(m_indexer.setSpindexerVelocityCmd(TestingDashboard.sub_spindexerVelocityRPS));
        TestingDashboard.trg_letTunnelVelocityRPSChange
            .whileTrue(m_indexer.setTunnelVelocityCmd(TestingDashboard.sub_tunnelVelocityRPS));

        TestingDashboard.trg_letIntakeArmPositionRotsChange
            .whileTrue(m_intake.setIntakeArmPos(TestingDashboard.sub_intakeArmPositionRots));
        TestingDashboard.trg_letIntakeRollersVelocityRPSChange
            .whileTrue(m_intake.setIntakeRollersVelocity(TestingDashboard.sub_intakeRollersVelocityRPS));
    }

    /* PERIODICS */
    @Override
    public void robotPeriodic() {
        m_periodicTracer.addEpoch("Entry (Unused Time)");
        CommandScheduler.getInstance().run(); 
        m_periodicTracer.addEpoch("CommandScheduler");


        for (var camera : WaltCamera.AllCameras) {
            Optional<EstimatedRobotPose> estimatedPoseOptional = camera.getEstimatedGlobalPose();
            if (estimatedPoseOptional.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
                Pose2d estimatedRobotPose2d = estimatedRobotPose.estimatedPose.toPose2d();
                var ctreTime = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
                m_drivetrain.addVisionMeasurement(estimatedRobotPose2d, estimatedRobotPose.timestampSeconds, camera.getEstimationStdDevs());
                m_visionSeenLastSec = ctreTime;
                // System.out.println("AddMeasurementFrom: " + camera.getName());
            }
        }
        m_periodicTracer.addEpoch("VisionUpdate");

        log_povUp.accept(m_driver.povUp());
        log_povDown.accept(m_driver.povDown());
        log_povLeft.accept(m_driver.povLeft());
        log_povRight.accept(m_driver.povRight());
        log_manipLeftTrigger.accept(m_manipulator.leftTrigger());
        log_manipRightTrigger.accept(m_manipulator.rightTrigger());
        log_driverLeftTrigger.accept(m_driver.leftTrigger());
        log_driverLeftTrigger.accept(m_driver.leftTrigger());

        log_visionSeenPastSecond.accept((Utils.getCurrentTimeSeconds() - m_visionSeenLastSec) < 1.0);
        log_isDisabled.accept(trg_limitFPS);
        m_periodicTracer.addEpoch("Logging");

        log_miniPCCurrent.accept(m_PDH.getCurrent(kMiniPCChannel));

        // log_shooterDirection.accept(
        //     new Pose3d(
        //         m_drivetrain.getState().Pose
        //     ).plus(
        //         kTurretTransform
        //     ).plus(
        //         new Transform3d(
        //             new Translation3d(), new Rotation3d(
        //                 Rotations.of(0),
        //                 Rotations.of(-m_shooter.getHoodSimEncoder().getAngularPositionRotations()),
        //                 m_shooter.getTurret().getPosition().getValue()
        //             )
        //         )
        //     )
        // );

        /* for the mechanism2D in 3D, drag all 3 mechanisms2ds onto the robot pose
        and also log the shooter position pose */ 
        // m_periodicTracer.printEpochs();
    }

    private final Timer m_fpsLimitTimer = new Timer();
    private final Timer lastGotTagMsmtTimer = new Timer();

    @Override
    public void disabledInit() {
        m_fpsLimitTimer.restart();
        WaltCamera.setFpsLimit(true);
        // if (!DriverStation.isFMSAttached()) {
            // m_shooter.setTurretNeutralMode(NeutralModeValue.Coast);
            // m_intake.setIntakeArmNeutralMode(NeutralModeValue.Coast);
        // }
        m_waltAutonFactory.setAlliance(
            DriverStation.getAlliance().isPresent() && 
            DriverStation.getAlliance().get().equals(Alliance.Red)
        );

        AutonChooser.initialize(m_simpleAutonFactory);
    }


    @Override
    public void disabledPeriodic() {
        if (m_fpsLimitTimer.hasElapsed(3) && !WaltCamera.areCamsFpsLimited()) {
            WaltCamera.setFpsLimit(true);
            m_fpsLimitTimer.restart();
        }
    }

    @Override
    public void disabledExit() {
        // if (!DriverStation.isFMSAttached()) {
            // System.out.println("Re-enabling motor brakes");
            // m_shooter.setTurretNeutralMode(NeutralModeValue.Brake);
            // m_intake.setIntakeArmNeutralMode(NeutralModeValue.Brake);
        // }
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = AutonChooser.m_chooser.getSelected().autonCommand;

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        // AutonChooser.cleanup();
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
        
        CommandScheduler.getInstance().schedule(
            Commands.sequence(
                m_drivetrain.runOnce(m_drivetrain::seedFieldCentric),
                Commands.waitSeconds(1),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(kMaxTranslationSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                ),
                Commands.waitSeconds(2.5),
                m_drivetrain.xBrake(),
                Commands.waitSeconds(2.5),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(kMaxTranslationSpeed.times(-1))
                        .withVelocityY(0)
                        .withRotationalRate(0)
                ),
                Commands.waitSeconds(2.5),
                m_drivetrain.xBrake(),
                Commands.waitSeconds(2.5),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(kDriverMaxAngularRate)
                ),
                Commands.waitSeconds(2.5),
                m_drivetrain.xBrake(),
                Commands.waitSeconds(2.5),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                )
            )
        );
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

        // m_visionSim.simulationPeriodic(robotPose);
        m_drivetrain.simulationPeriodic();
        m_shooter.simulationPeriodic();
        m_intake.simulationPeriodic();
        m_indexer.simulationPeriodic();
    }
}
