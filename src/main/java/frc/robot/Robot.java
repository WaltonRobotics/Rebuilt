
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeK.kIntakeRollersIntakeVolts;
import static frc.robot.Constants.RobotK.*;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCalc;
import frc.robot.Constants.RobotK;
import frc.robot.Constants.ShooterK;
import frc.robot.dashboards.AutonChooser;
import frc.robot.autons.WaltAdaptableAutonFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer;
import frc.robot.vision.WaltCamera;
import frc.util.HubShiftUtil;
import frc.util.PerformanceMonitor;
import frc.util.SignalManager;
// import frc.util.WaltVisualSim;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose2dLogger;

public class Robot extends TimedRobot {
    /* CLASS VARIABLES */
    //---CONSTANTS
    private final LinearVelocity kMaxTranslationSpeed = TunerConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    private final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(1.05); // 3/4 of a rotation per second max angular velocity

    // Pre-computed doubles for driveCommand hot path (avoids .times() measure allocations every tick)
    private final double kMaxTranslationMps = kMaxTranslationSpeed.in(MetersPerSecond);
    private final double kMaxAngularRps = kMaxAngularRate.in(RadiansPerSecond);

    private double m_visionSeenLastSec = Utils.getCurrentTimeSeconds();
    private final BooleanLogger log_visionSeenPastSecond = new BooleanLogger(kLogTab, "VisionSeenLastSec");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxTranslationSpeed.times(0.1)).withRotationalDeadband(kMaxAngularRate.times(0.1)) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final SwerveRequest.RobotCentric tuneDrive = new SwerveRequest.RobotCentric()
    //     .withDeadband(kMaxTranslationSpeed.times(0.1)).withRotationalDeadband(kMaxAngularRate.times(0.1)) // Add a 10% deadband
    //     .withDriveRequestType(DriveRequestType.Velocity);

    // private final Telemetry logger = new Telemetry(kMaxTranslationSpeed.in(MetersPerSecond));

    //---CONTROLLERS
    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_manipulator = new CommandXboxController(1);

    //---INIT SUBSYSTEMS
    public final Swerve m_drivetrain = TunerConstants.createDrivetrain();

    private final Shooter m_shooter = new Shooter(
        () -> m_drivetrain.getState().Pose, 
        () -> m_drivetrain.getStateCopy(),
        () -> m_drivetrain.getChassisSpeeds());

    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();

    // private final WaltVisualSim m_visualSim;
    private final Superstructure m_superstructure = new Superstructure(m_intake, m_indexer, m_shooter);

    //---AUTONS
    private final AutoFactory m_autoFactory = m_drivetrain.createAutoFactory();
    private final WaltAdaptableAutonFactory m_adpatableAutonFactory = new WaltAdaptableAutonFactory(m_superstructure, m_autoFactory, m_intake, m_shooter, m_drivetrain);
    //---VISION

    private PowerDistribution m_PDH = new PowerDistribution();
    // private final NetworkPinger m_radioPinger = new NetworkPinger("Radio", "10.29.74.1", 0.2, 10);
    // private final NetworkPinger m_coprocessorPinger = new NetworkPinger("Coprocessor", "10.29.74.11", 0.2, 10);
    // private final VisionSim m_visionSim = new VisionSim();

    /* TRIGGERS */
    // private Trigger trg_optimalPrefireTime = new Trigger(HubShiftUtil.optimalPrefireTime());
    // private Trigger trg_comebackTime = new Trigger(HubShiftUtil.comebackTime());
    private Trigger trg_snappingBack = new Trigger(ShooterCalc.isSnappingBack());
    private Trigger trg_driverOverride = m_driver.b();
    private Trigger trg_manipOverride = m_manipulator.b();

    //---COMMAND SEQUENCE TRIGGERS
    private Trigger trg_intake = m_manipulator.rightTrigger().and(trg_manipOverride.negate());
    private Trigger trg_retractIntake = m_manipulator.rightBumper().and(trg_manipOverride.negate());

    private Trigger trg_shoot = m_driver.rightTrigger().and(trg_driverOverride.negate());
    private Trigger trg_emergencyBarf = m_driver.rightTrigger().and(trg_driverOverride);

    private Trigger trg_intakeShimmy = m_manipulator.leftBumper();

    private Trigger trg_emergencyIntakeBarf = m_manipulator.rightTrigger().and(trg_manipOverride);

    //---OVERRIDE TRIGGERS
    private final Trigger trg_limitFPS = RobotModeTriggers.disabled();
    private final Trigger trg_unlimitFps = RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop());

    private Trigger trg_unjam = m_driver.rightBumper();

    private final DoubleLogger log_miniPCCurrent = WaltLogger.logDouble(kLogTab, "MiniPC current");
    private final DoubleLogger log_rioBusVoltage = WaltLogger.logDouble(kLogTab, "RioBusVoltage");
    private final BooleanLogger log_rioBrownout = WaltLogger.logBoolean(kLogTab, "RioBrownout");
    private final DoubleLogger log_pdhCurrentTotal = WaltLogger.logDouble(kLogTab, "PDHCurrTotal");
    private final BooleanLogger log_isDSAttatched = WaltLogger.logBoolean(kLogTab, "isDSAttatched");
    private final Pose2dLogger log_robotPose = WaltLogger.logPose2d("Drive", "Pose");
    private final BooleanLogger log_isBeached = WaltLogger.logBoolean("Drive", "isBeached");

    private final DoubleLogger log_autonTime = WaltLogger.logDouble("Auton", "autonTime");

    //DRIVERSTATION LOGS TELL US
    // private final BooleanLogger log_isDisabled = WaltLogger.logBoolean(kLogTab, "is robot disabled");

    //NOT DISPLAYING
    // private final StringLogger log_currentShift = WaltLogger.logString("Util/Shift", "currentShift");
    // private final DoubleLogger log_elapsedTime = WaltLogger.logDouble("Util/Shift", "elapsedTime");
    // private final DoubleLogger log_remainingTime = WaltLogger.logDouble("Util/Shift", "currentTime");
    // private final BooleanLogger log_isActive = WaltLogger.logBoolean("Util/Shift", "isActive");
    // private final StringLogger log_currentFudgedShift = WaltLogger.logString("Util/Shift", "currentFudgedShift");
    // private final DoubleLogger log_elapsedFudgedTime = WaltLogger.logDouble("Util/Shift", "elapsedFudgedTime");
    // private final DoubleLogger log_remainingFudgedTime = WaltLogger.logDouble("Util/Shift", "currentFudgedTime");
    // private final BooleanLogger log_isActiveFudged = WaltLogger.logBoolean("Util/Shift", "isActiveFudged");

    // private final Tracer m_periodicTracer = new Tracer();
    private final PerformanceMonitor m_perfMonitor = new PerformanceMonitor(false);
    private final Command m_preheaterCommand;

    /* CONSTRUCTOR */
    public Robot() {
        configureBindings();
        // configureTestBindings();    //this should be commented out during competition matches

        lastGotTagMsmtTimer.start();

        AutonChooser.initialize(m_adpatableAutonFactory);

        // Choreo warmup. Runs synchronously on the main thread during robotInit so
        // class-loading, trajectory JSON parsing, and routine/trigger composition all
        // happen up-front instead of on the first autonomousInit tick.
        AutonChooser.forceLoadChoreoClasses();
        m_adpatableAutonFactory.preloadAllTrajectories(AutonChooser.allTrajectoryNames());
        AutonChooser.preheatAllRoutines();

        RobotModeTriggers.autonomous().whileTrue(
            AutonChooser.m_chooser.selectedCommandScheduler().withTimeout(20.3)
        );

        // set FPS limit on boot
        WaltCamera.setFpsLimit(true);

        DriverStation.silenceJoystickConnectionWarning(true);
        PhotonCamera.setVersionCheckEnabled(false);
        LiveWindow.disableAllTelemetry();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        // MANUAL HOMING IS BEING USED
        // addPeriodic(m_shooter::fastPeriodic, 0.0025);


        m_preheaterCommand = AutonChooser.getPreheater();
        CommandScheduler.getInstance().schedule(m_preheaterCommand);
    }

    /* COMMANDS */
    /**
     * 
     * @param speedMult how much you want to limit speed as a decimal percentage of kMaxTranslation. 1 does nothing
     * @return swerve drive command
     */
    private Command driveCommand(double speedMult, double rotationMult) {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will execute this command periodically
        final double slowMps = kMaxTranslationMps * speedMult;
        final double slowRotRps = kMaxAngularRps * rotationMult;
        return m_drivetrain.applyRequest(() -> {
            double translationMps = m_driver.leftTrigger().getAsBoolean() ? slowMps : kMaxTranslationMps;
            double rotationalMps = m_driver.leftTrigger().getAsBoolean() ? slowRotRps : kMaxAngularRps;

            double driverXVelo = translationMps * -m_driver.getLeftY();
            double driverYVelo = translationMps * -m_driver.getLeftX();
            double driverYawRate = rotationalMps * -m_driver.getRightX(); //m_driver.leftBumper().getAsBoolean()
                // ? slowRotRps * -m_driver.getRightX()
                // : kMaxAngularRps * -m_driver.getRightX();

            return drive
                .withVelocityX(driverXVelo) // Drive forward with Y (forward)
                .withVelocityY(driverYVelo) // Drive left with X (left)
                .withRotationalRate(driverYawRate); // Drive counterclockwise with negative X (left)
            }
        );
    }

    // private void setBothRumble(RumbleType type, double intensity) {
    //     m_driver.setRumble(type, intensity);
    //     m_manipulator.setRumble(type, intensity);
    // }

    //---BINDINGS
    private void configureBindings() {
        /* GENERATED SWERVE BINDS */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(driveCommand(RobotK.kRobotSpeedIntakingLimit, RobotK.kRobotEvasionLimit));

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

        // m_driver.y().whileTrue(m_drivetrain.applyRequest(() -> tuneDrive.withVelocityX(0.5)));
        // m_driver.x().whileTrue(m_drivetrain.applyRequest(() -> tuneDrive.withVelocityX(-0.5)));
        // m_driver.a().whileTrue(m_drivetrain.applyRequest(() -> tuneDrive.withVelocityX(0)));

        // m_drivetrain.registerTelemetry(logger::telemeterize);

        /* CUSTOM BINDS */
        trg_limitFPS.onTrue(WaltCamera.setFpsLimitCmd(true));   
        trg_unlimitFps.onTrue(WaltCamera.setFpsLimitCmd(false));

        // robot heads toward fuel when detected :D (hypothetically)(robo could blow up instead)
        // trg_swerveToObject.whileTrue(
        //     m_drivetrain.swerveToObject()
        // );

        //---NORMAL SEQUENCES
        //Intake
        trg_intake.and(trg_shoot.negate()).and(trg_emergencyBarf.negate()).whileTrue(
            m_superstructure.intake(() -> false, () -> false)
        );

        // trg_retractIntake.onTrue(
        //     m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        // );

        //Shooting
        // NORMAL FIXED SHOT
        // trg_turretInShootRange.whileFalse(Commands.run(() -> m_driver.setRumble(RumbleType.kBothRumble, 0.3)).finallyDo(() -> m_driver.setRumble(RumbleType.kBothRumble, 0)));

        // DRIVER SHOOTING 
        trg_shoot
            .and(() -> m_shooter.m_turret.atPosition())
            .and(trg_snappingBack.negate())
            .whileTrue(m_superstructure.activateOuttakeShotCalc());

        trg_shoot
            .and(trg_snappingBack)
            .whileTrue(m_superstructure.hoodAndFlywheelShotCalc());
        // m_manipulator.rightTrigger().and(trg_manipOverride).whileTrue(Commands.run(() -> m_intake.setIntakeRollersVelocity(kIntakeRollersBarfVolts)).finallyDo(() -> m_intake.setIntakeRollersVelocity(0)));

        // m_driver.y().onTrue(m_shooter.driverRPSAlter(true));
        // m_driver.a().onTrue(m_shooter.driverRPSAlter(false));

        // m_driver.x().onTrue(m_shooter.driverResetRPSAlter());

        // m_driver.leftBumper().whileTrue(m_shooter.driverRPSIncreaseWhileHeldCmd());

        // snapshot on each shoot press
        trg_shoot.onTrue(WaltCamera.takeSnapshotCmd());

        trg_intake.and(trg_shoot).and(trg_emergencyBarf.negate()).whileTrue(
            m_superstructure.intake(() -> true, () -> false)
        );

        trg_retractIntake.onTrue(m_superstructure.retractIntake());

        trg_emergencyBarf.whileTrue(
            m_superstructure.emergencyBarf()
        );

        trg_emergencyIntakeBarf.whileTrue(
            m_superstructure.emergencyBarfNOSHOOT()
        );
        
        trg_intakeShimmy.whileTrue(m_superstructure.intakeShimmy(() -> false));

        trg_intakeShimmy.and(trg_shoot).whileTrue(m_superstructure.intakeShimmy(() -> true));

        trg_unjam.and(trg_shoot.negate()).whileTrue(
            m_superstructure.unjamCmd(() -> false)
        );

        trg_unjam.and(trg_shoot).whileTrue(
            m_superstructure.unjamCmd(()-> true)
        );

        //---OVERRIDE COMMANDS
        m_manipulator.x().and(trg_manipOverride).onTrue(m_intake.intakeArmCurrentSenseHoming());

        m_manipulator.start().and(trg_manipOverride).onTrue(m_shooter.m_hood.hoodCurrentSenseHomingCmd());
        m_manipulator.leftTrigger().onTrue(m_shooter.m_hood.setHoodPosCmd(ShooterK.kHoodMaxRots_double / 2.0));

        // m_driver.y().and(trg_driverOverride).onTrue(m_shooter.turretHomingCmd(false));  //false? im not sure

        m_driver.povDown().onTrue(m_shooter.m_turret.setTurretLockCmd(false));
        m_driver.povRight().onTrue(m_shooter.m_turret.setTurretLockCmd(true));

        m_manipulator.y().and(trg_manipOverride).onTrue(Commands.runOnce(() -> m_shooter.m_turret.homeTurret(true)));
        
        // m_driver.povDown().onTrue(m_drivetrain.roboToTranslation(new Translation2d(m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY() - Inches.of(20).magnitude()), 0.001));
        // m_driver.povUp().onTrue(m_drivetrain.roboToTranslation(new Translation2d(m_drivetrain.getState().Pose.getX(), m_drivetrain.getState().Pose.getY() + Inches.of(20).magnitude()), 0.001));
        // m_driver.povRight().onTrue(m_drivetrain.roboToTranslation(new Translation2d(m_drivetrain.getState().Pose.getX() + Inches.of(20).magnitude(), m_drivetrain.getState().Pose.getY()), 0.001));
        // m_driver.povLeft().onTrue(m_drivetrain.roboToTranslation(new Translation2d(m_drivetrain.getState().Pose.getX() - Inches.of(20).magnitude(), m_drivetrain.getState().Pose.getY()), 0.001));

        // m_driver.start().whileTrue(m_superstructure.activateOuttakeNOSHOOT());
        // trg_optimalPrefireTime.whileTrue(
        //     Commands.run(() -> setBothRumble(RumbleType.kBothRumble, 0.5)).finallyDo(() -> setBothRumble(RumbleType.kBothRumble, 0))
        // );

        // trg_comebackTime.whileTrue(
        //     Commands.run(() -> setBothRumble(RumbleType.kRightRumble, 0.5)).finallyDo(()-> setBothRumble(RumbleType.kRightRumble, 0))
        // );
    }

    private void configureTestBindings() {}

    /* PERIODICS */
    @Override
    public void robotPeriodic() {
        m_perfMonitor.loopStart();
        // m_periodicTracer.addEpoch("Entry (Unused Time)");
        SignalManager.refreshAll();
        CommandScheduler.getInstance().run();
        // m_periodicTracer.addEpoch("CommandScheduler");

        log_robotPose.accept(m_drivetrain.getState().Pose);

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
        // m_periodicTracer.addEpoch("VisionUpdate");

        log_visionSeenPastSecond.accept((Utils.getCurrentTimeSeconds() - m_visionSeenLastSec) < 1.0);
        // log_isDisabled.accept(trg_limitFPS);
        // m_periodicTracer.addEpoch("Logging");

        log_miniPCCurrent.accept(m_PDH.getCurrent(kMiniPCChannel));
        log_rioBusVoltage.accept(RobotController.getBatteryVoltage());
        log_rioBrownout.accept(RobotController.isBrownedOut());
        log_pdhCurrentTotal.accept(m_PDH.getTotalCurrent());
        log_isDSAttatched.accept(DriverStation.isDSAttached());
        log_isBeached.accept(m_drivetrain.isBeached());

        // log_currentShift.accept(HubShiftUtil.getOfficialShiftInfo().currentShift().toString());
        // log_currentFudgedShift.accept(HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
        // log_elapsedTime.accept(Math.floor(HubShiftUtil.getOfficialShiftInfo().elapsedTime() * 100) / 100.0);
        // log_elapsedFudgedTime.accept((Math.floor(HubShiftUtil.getShiftedShiftInfo().elapsedTime() * 100) / 100.0));
        // log_remainingTime.accept((Math.floor(HubShiftUtil.getOfficialShiftInfo().remainingTime() * 100) / 100.0));
        // log_remainingFudgedTime.accept((Math.floor(HubShiftUtil.getShiftedShiftInfo().remainingTime() * 100) / 100.0));
        // log_isActive.accept(() -> HubShiftUtil.getOfficialShiftInfo().active());
        // log_isActiveFudged.accept(() -> HubShiftUtil.getShiftedShiftInfo().active());

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
        m_perfMonitor.loopEnd();
    }

    private final Timer m_fpsLimitTimer = new Timer();
    private final Timer lastGotTagMsmtTimer = new Timer();
    private final Timer m_disableChangeDelayTimer = new Timer();

    @Override
    public void disabledInit() {
        WaltLogger.timedPrint("Robot::disabledInit");
        m_disableChangeDelayTimer.restart();
    }


    @Override
    public void disabledPeriodic() {
        if (m_fpsLimitTimer.hasElapsed(3) && !WaltCamera.areCamsFpsLimited()) {
            WaltCamera.setFpsLimit(true);
            m_fpsLimitTimer.restart();
            // dumb bullshit to hot-path the Chooser on occasion

        }

        // oneshot on DisabledInit
        if (m_disableChangeDelayTimer.hasElapsed(3.0)) {
            m_disableChangeDelayTimer.stop();
            m_disableChangeDelayTimer.reset();
            m_shooter.m_turret.setTurretNeutralMode(NeutralModeValue.Coast);
            m_intake.setIntakeArmNeutralMode(NeutralModeValue.Coast);
            m_shooter.m_hood.setHoodNeutralMode(NeutralModeValue.Coast);
        }
    }

    @Override
    public void disabledExit() {
        WaltLogger.timedPrint("Robot::disabledExit");
        m_disableChangeDelayTimer.restart();
    }

    @Override
    public void autonomousInit() {
        m_adpatableAutonFactory.startAutonTimer();
    }

    @Override
    public void autonomousPeriodic() {
        // m_adpatableAutonFactory.logTimer("Auton", () -> m_adpatableAutonFactory.autonTimer);
        log_autonTime.accept(m_adpatableAutonFactory.autonTimer.get());
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        HubShiftUtil.initialize();
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
                m_drivetrain.xBrakeCmd(),
                Commands.waitSeconds(2.5),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(kMaxTranslationSpeed.unaryMinus())
                        .withVelocityY(0)
                        .withRotationalRate(0)
                ),
                Commands.waitSeconds(2.5),
                m_drivetrain.xBrakeCmd(),
                Commands.waitSeconds(2.5),
                m_drivetrain.applyRequest(() ->
                    drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(kMaxAngularRate)
                ),
                Commands.waitSeconds(2.5),
                m_drivetrain.xBrakeCmd(),
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
        // FuelSim.getInstance().start();
    }

    @Override
    public void simulationPeriodic() {
        // FuelSim instance = FuelSim.getInstance();
        // instance.logFuels();
        // instance.updateSim();

        SwerveDriveState robotState = m_drivetrain.getState();
        Pose2d robotPose = robotState.Pose;
        WaltCamera.m_visionSim.simulationPeriodic(robotPose);
        m_drivetrain.simulationPeriodic();
        m_shooter.simulationPeriodic();
        m_intake.simulationPeriodic();
        m_indexer.simulationPeriodic();
    }
}
