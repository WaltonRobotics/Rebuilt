
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FieldK.kLeftResetPose;
import static frc.robot.Constants.FieldK.kRightResetPose;
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
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.shooter.Shooter;
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
    // Captured before any field initializers run (subsystem construction, etc.)
    private static final long kFieldInitStart = System.nanoTime();

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
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final SwerveRequest.RobotCentric tuneDrive = new SwerveRequest.RobotCentric()
    //     .withDeadband(kMaxTranslationSpeed.times(0.1)).withRotationalDeadband(kMaxAngularRate.times(0.1)) // Add a 10% deadband
    //     .withDriveRequestType(DriveRequestType.Velocity);

    // private final Telemetry logger = new Telemetry(kMaxTranslationSpeed.in(MetersPerSecond));

    private final SlewRateLimiter limit_driverX = new SlewRateLimiter(0.30);
    private final SlewRateLimiter limit_driverY = new SlewRateLimiter(0.30);
    private final SlewRateLimiter limit_driverYawRate = new SlewRateLimiter(0.30);

    //---CONTROLLERS
    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_manipulator = new CommandXboxController(1);

    // Cached so the drive default-command lambda doesn't allocate a Trigger every tick.
    private final Trigger trg_driverSlow = m_driver.leftTrigger();

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
    private final Trigger trg_snappingBack = new Trigger(m_shooter.m_turret.isSnappingBack());
    private final Trigger trg_driverOverride = m_driver.b();
    private final Trigger trg_manipOverride = m_manipulator.b();

    //---DRIVER BUTTONS
    private final Trigger trg_shoot = m_driver.rightTrigger().and(trg_driverOverride.negate());
    private final Trigger trg_emergencyBarf = m_driver.rightTrigger().and(trg_driverOverride);
    private final Trigger trg_unjam = m_driver.rightBumper();
    private final Trigger trg_resetPoseLeft = m_driver.leftBumper().and(trg_driverOverride.and(m_driver.povLeft()));
    private final Trigger trg_resetPoseRight = m_driver.leftBumper().and(trg_driverOverride.and(m_driver.povRight()));

    private final Trigger trg_lockShooting = m_driver.povRight();
    private final Trigger trg_unlockShooting = m_driver.povDown();

    //---MANIPULATOR BUTTONS
    private final Trigger trg_intake = m_manipulator.rightTrigger().and(trg_manipOverride.negate());
    private final Trigger trg_retractIntake = m_manipulator.rightBumper().and(trg_manipOverride.negate());
    private final Trigger trg_intakeShimmy = m_manipulator.leftBumper();

    private final Trigger trg_emergencyIntakeOnlyBarf = m_manipulator.rightTrigger().and(trg_manipOverride);

    private final Trigger trg_homeIntake =  m_manipulator.x().and(trg_manipOverride);
    private final Trigger trg_homeHood = m_manipulator.start().and(trg_manipOverride);
    private final Trigger trg_reseedTurret = m_manipulator.y().and(trg_manipOverride);

    //---MISC TRGs
    private final Trigger trg_limitFPS = RobotModeTriggers.disabled();
    private final Trigger trg_unlimitFps = RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop());



    private final DoubleLogger log_miniPCCurrent = WaltLogger.logDouble(kLogTab, "MiniPC current");
    private final DoubleLogger log_rioBusVoltage = WaltLogger.logDouble(kLogTab, "RioBusVoltage");
    private final BooleanLogger log_rioBrownout = WaltLogger.logBoolean(kLogTab, "RioBrownout");
    private final DoubleLogger log_pdhCurrentTotal = WaltLogger.logDouble(kLogTab, "PDHCurrTotal");
    private final BooleanLogger log_isDSAttatched = WaltLogger.logBoolean(kLogTab, "isDSAttatched");
    private final Pose2dLogger log_robotPose = WaltLogger.logPose2d("Drive", "Pose", true);

    private final DoubleLogger log_autonTime = WaltLogger.logDouble("Auton", "autonTime");

    //NOT DISPLAYING
    // private final StringLogger log_currentShift = WaltLogger.logString("Util/Shift", "currentShift");
    // private final DoubleLogger log_elapsedTime = WaltLogger.logDouble("Util/Shift", "elapsedTime");
    // private final DoubleLogger log_remainingTime = WaltLogger.logDouble("Util/Shift", "currentTime");
    // private final BooleanLogger log_isActive = WaltLogger.logBoolean("Util/Shift", "isActive");
    // private final StringLogger log_currentFudgedShift = WaltLogger.logString("Util/Shift", "currentFudgedShift");
    // private final DoubleLogger log_elapsedFudgedTime = WaltLogger.logDouble("Util/Shift", "elapsedFudgedTime");
    // private final DoubleLogger log_remainingFudgedTime = WaltLogger.logDouble("Util/Shift", "currentFudgedTime");
    // private final BooleanLogger log_isActiveFudged = WaltLogger.logBoolean("Util/Shift", "isActiveFudged");

    private final Tracer m_periodicTracer = new Tracer();
    private final PerformanceMonitor m_perfMonitor = new PerformanceMonitor(false);
    private final Command m_preheaterCommand;

    /* CONSTRUCTOR */
    public Robot() {
        long t0 = System.nanoTime();
        long tPrev = t0;
        System.out.printf("[INIT PROFILE] field initializers (subsystems, HW): %7.1f ms%n", (t0 - kFieldInitStart) * 1e-6);

        configureBindings();
        // configureTestBindings();    //this should be commented out during competition matches
        long tBindings = System.nanoTime();
        System.out.printf("[INIT PROFILE] configureBindings:        %7.1f ms%n", (tBindings - tPrev) * 1e-6);
        tPrev = tBindings;

        lastGotTagMsmtTimer.start();

        AutonChooser.initialize(m_adpatableAutonFactory);
        long tChooserInit = System.nanoTime();
        System.out.printf("[INIT PROFILE] AutonChooser.initialize:  %7.1f ms%n", (tChooserInit - tPrev) * 1e-6);
        tPrev = tChooserInit;

        // Choreo warmup. Runs synchronously on the main thread during robotInit so
        // class-loading, trajectory JSON parsing, and routine/trigger composition all
        // happen up-front instead of on the first autonomousInit tick.
        AutonChooser.forceLoadChoreoClasses();
        long tClassLoad = System.nanoTime();
        System.out.printf("[INIT PROFILE] forceLoadChoreoClasses:   %7.1f ms%n", (tClassLoad - tPrev) * 1e-6);
        tPrev = tClassLoad;

        // m_adpatableAutonFactory.preloadAllTrajectories(AutonChooser.allTrajectoryNames());
        // long tPreload = System.nanoTime();
        // System.out.printf("[INIT PROFILE] preloadAllTrajectories:   %7.1f ms%n", (tPreload - tPrev) * 1e-6);
        // tPrev = tPreload;

        // AutonChooser.preheatAllRoutines();
        // long tPreheat = System.nanoTime();
        // System.out.printf("[INIT PROFILE] preheatAllRoutines:       %7.1f ms%n", (tPreheat - tPrev) * 1e-6);
        // tPrev = tPreheat;

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

        long tMisc = System.nanoTime();
        System.out.printf("[INIT PROFILE] misc (cameras/logging):   %7.1f ms%n", (tMisc - tPrev) * 1e-6);
        tPrev = tMisc;

        m_preheaterCommand = AutonChooser.getPreheater();
        CommandScheduler.getInstance().schedule(m_preheaterCommand);
        // m_preheaterCommand = AutonChooser.m_chooser.selectedCommandScheduler();
        // CommandScheduler.getInstance().schedule(m_preheaterCommand);

        long tEnd = System.nanoTime();
        System.out.printf("[INIT PROFILE] preheater cmd build:      %7.1f ms%n", (tEnd - tPrev) * 1e-6);
        System.out.printf("[INIT PROFILE] =============================%n");
        System.out.printf("[INIT PROFILE] CONSTRUCTOR TOTAL:        %7.1f ms%n", (tEnd - t0) * 1e-6);
    }

    /* COMMANDS */
    /**
     * 
     * @param speedMult how much you want to limit speed as a decimal percentage of kMaxTranslation. 1 does nothing
     * @return swerve drive command
     */
    private Command driveCommand(double speedMult, double rotationMult) {
        // X=Forward, Y=Left
        // Drivetrain will execute this command periodically
        final double slowMps = kMaxTranslationMps * speedMult;
        final double slowRotRps = kMaxAngularRps * rotationMult;

        return m_drivetrain.applyRequest(() -> {
            boolean slowButton = trg_driverSlow.getAsBoolean();
            double translationMps = slowButton ? slowMps : kMaxTranslationMps;
            double rotationalMps = slowButton ? slowRotRps : kMaxAngularRps;

            double driverXVelo = translationMps * -m_driver.getLeftY();
            double driverYVelo = translationMps * -m_driver.getLeftX();
            double driverYawRate = rotationalMps * -m_driver.getRightX(); //m_driver.leftBumper().getAsBoolean()
                // ? slowRotRps * -m_driver.getRightX()
                // : kMaxAngularRps * -m_driver.getRightX();

            return drive
                .withVelocityX(slowButton ? limit_driverX.calculate(driverXVelo) : driverXVelo) // Drive forward with Y (forward)
                .withVelocityY(slowButton ? limit_driverY.calculate(driverYVelo) : driverYVelo) // Drive left with X (left)
                .withRotationalRate(slowButton ? limit_driverYawRate.calculate(driverYawRate) : driverYawRate); // Drive counterclockwise with negative X (left)
            }
        );
    }

    // private void setBothRumble(RumbleType type, double intensity) {
    //     m_driver.setRumble(type, intensity);
    //     m_manipulator.setRumble(type, intensity);
    // }

    //---BINDINGS
    private void configureBindings() {
        /* SET UP */
        m_drivetrain.setDefaultCommand(driveCommand(RobotK.kRobotSpeedIntakingLimit, RobotK.kRobotEvasionLimit));

        // Idle while the robot is disabled. This ensures the configured neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        trg_limitFPS.onTrue(WaltCamera.setFpsLimitCmd(true));   
        trg_unlimitFps.onTrue(WaltCamera.setFpsLimitCmd(false));

        /* BUTTON BIDNDS */
        m_driver.leftBumper().and(trg_driverOverride).onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));    // Reset the field-centric heading on left bumper press.

        trg_shoot
            .and(() -> m_shooter.m_turret.atPosition())
            .and(trg_snappingBack.negate())
            .whileTrue(m_superstructure.activateOuttakeShotCalc());

        trg_shoot
            .and(trg_snappingBack)
            .whileTrue(m_superstructure.hoodAndFlywheelShotCalc());

        // snapshot on each shoot press
        trg_shoot.onTrue(WaltCamera.takeSnapshotCmd());

        trg_intake.and(trg_shoot).and(trg_emergencyBarf.negate()).whileTrue(
            m_superstructure.intake(() -> true, () -> false)
        );

        trg_intake.and(trg_shoot.negate()).and(trg_emergencyBarf.negate()).whileTrue(
            m_superstructure.intake(() -> false, () -> false)
        );

        trg_retractIntake.onTrue(m_superstructure.retractIntake());
        trg_intakeShimmy.whileTrue(m_superstructure.intakeShimmy(() -> false));
        trg_intakeShimmy.and(trg_shoot).whileTrue(m_superstructure.intakeShimmy(() -> true));

        trg_emergencyBarf.whileTrue(m_superstructure.emergencyBarf());
        trg_emergencyIntakeOnlyBarf.whileTrue(m_superstructure.emergencyBarfOnlyIntake());

        trg_emergencyBarf.whileTrue(m_superstructure.emergencyBarf());
        trg_emergencyIntakeOnlyBarf.whileTrue(m_superstructure.emergencyBarfOnlyIntake());

        trg_unjam.and(trg_shoot.negate()).whileTrue(m_superstructure.unjamCmd(() -> false));
        trg_unjam.and(trg_shoot).whileTrue(m_superstructure.unjamCmd(()-> true));

        trg_homeIntake.onTrue(m_intake.intakeArmCurrentSenseHoming());
        trg_homeHood.onTrue(m_shooter.m_hood.hoodCurrentSenseHomingCmd());
        trg_reseedTurret.onTrue(Commands.runOnce(() -> m_shooter.m_turret.homeTurret(true)));

        trg_lockShooting.onTrue(m_shooter.m_turret.setTurretLockCmd(true));
        trg_unlockShooting.onTrue(m_shooter.m_turret.setTurretLockCmd(false));

        trg_resetPoseLeft.onTrue(Commands.runOnce(() -> m_drivetrain.resetPose(kLeftResetPose)));
        trg_resetPoseRight.onTrue(Commands.runOnce(() -> m_drivetrain.resetPose(kRightResetPose)));

        //---OLD RUMBLE LOGIC
        // Trigger trg_hubActiveOrPassing =
        //     new Trigger(
        //         () ->
        //             HubShiftUtil.getShiftedShiftInfo().active()
        //                 || m_shooter.getCurrentGoal().equals(ShooterGoal.PASSING));
        
        // trg_optimalPrefireTime.whileTrue(
        //     Commands.run(() -> setBothRumble(RumbleType.kBothRumble, 0.5)).finallyDo(() -> setBothRumble(RumbleType.kBothRumble, 0))
        // );

        // trg_comebackTime.whileTrue(
        //     Commands.run(() -> setBothRumble(RumbleType.kRightRumble, 0.5)).finallyDo(()-> setBothRumble(RumbleType.kRightRumble, 0))
        // );

        // m_drivetrain.registerTelemetry(logger::telemeterize);    //UNUSED - runs at 250hz which is burning CPU
      
        //-used when the shooter couldn't shoot while aiming close to the hopper wall
        // trg_turretInShootRange.whileFalse(Commands.run(() -> m_driver.setRumble(RumbleType.kBothRumble, 0.3))
        //     .finallyDo(() -> m_driver.setRumble(RumbleType.kBothRumble, 0))
    }

    private void configureTestBindings() {
        m_driver.povLeft().onTrue(m_shooter.m_hood.setHoodPosCmd(ShooterK.kHoodMinRots_double));
        m_driver.povUp().onTrue(m_shooter.m_hood.setHoodPosCmd(ShooterK.kHoodMaxRots_double));

        //---OTHER POSSIBLE BUTTON BINDS
        // m_driver.y().onTrue(m_shooter.driverRPSAlter(true));
        // m_driver.a().onTrue(m_shooter.driverRPSAlter(false));

        // m_driver.x().onTrue(m_shooter.driverResetRPSAlter());

        // m_driver.leftBumper().whileTrue(m_shooter.driverRPSIncreaseWhileHeldCmd());

        //robot heads toward fuel when detected :D (hypothetically)(robo could blow up instead)
        // trg_swerveToObject.whileTrue(
        //     m_drivetrain.swerveToObject()
        // );
    }

    /* PERIODICS */
    @Override
    public void robotPeriodic() {
        m_perfMonitor.loopStart();
        m_periodicTracer.addEpoch("Entry (Unused Time)");
        SignalManager.refreshAll();
        CommandScheduler.getInstance().run();
        m_periodicTracer.addEpoch("CommandScheduler");

        SwerveDriveState driveState = m_drivetrain.getState();
        log_robotPose.accept(driveState.Pose);

        var headingNow = driveState.Pose.getRotation();
        double nowSec = Utils.getCurrentTimeSeconds();
        for (var camera : WaltCamera.AllCameras) {
            camera.m_estimator.addHeadingData(nowSec, headingNow);
            Optional<EstimatedRobotPose> estimatedPoseOptional = camera.getEstimatedGlobalPose();
            if (estimatedPoseOptional.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
                Pose2d estimatedRobotPose2d = estimatedRobotPose.estimatedPose.toPose2d();
                m_drivetrain.addVisionMeasurement(estimatedRobotPose2d, estimatedRobotPose.timestampSeconds, camera.getEstimationStdDevs());
                m_visionSeenLastSec = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
                // System.out.println("AddMeasurementFrom: " + camera.getName());
            }
        }
        m_periodicTracer.addEpoch("VisionUpdate");

        log_visionSeenPastSecond.accept((nowSec - m_visionSeenLastSec) < 1.0);
        // log_isDisabled.accept(trg_limitFPS);
        m_periodicTracer.addEpoch("Logging");

        log_miniPCCurrent.accept(m_PDH.getCurrent(kMiniPCChannel));
        log_rioBusVoltage.accept(RobotController.getBatteryVoltage());
        log_rioBrownout.accept(RobotController.isBrownedOut());
        log_pdhCurrentTotal.accept(m_PDH.getTotalCurrent());
        log_isDSAttatched.accept(DriverStation.isDSAttached());

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


        m_periodicTracer.printEpochs();
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
