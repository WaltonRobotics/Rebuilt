package frc.robot.autons;

import static frc.robot.Constants.IntakeK.kIntakeRollersIntakeVolts;

import java.util.Set;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonK;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.WaltLogger;
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose2dArrayLogger;
import frc.util.WaltLogger.StringLogger;

public class WaltAdaptableAutonFactory {
    private final Superstructure m_superstructure;
    public final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Swerve m_drivetrain;

    public Timer autonTimer = new Timer();

    // state logger
    // private final DoubleLogger log_autonState = WaltLogger.logDouble(kLogTab, "State");

    // waypoint constants
    private final String kIntakeWaypoint = "intake";
    private final String kStopIntakeWaypoint = "stopIntake";
    private final String kShootWaypoint = "shoot";
    private final String kStopShootWaypoint = "stopShoot";
    private final String kIntakeAndShootWaypoint = "intakeAndShoot";
    private final String kStopIntakeAndShootWaypoint = "stopIntakeAndShoot";

    // trajectory logger
    private final StringLogger log_trajectoryName = new StringLogger(AutonK.kLogTab, "trajectoryName");
    private final Pose2dArrayLogger log_trajectoryPoses = new Pose2dArrayLogger(AutonK.kLogTab, "trajectoryPoses");
    private final BooleanLogger log_isAtStopShoot = new BooleanLogger(AutonK.kLogTab, "isAtStopShoot");
    private final BooleanLogger log_isAtStopIntake = new BooleanLogger(AutonK.kLogTab, "isAtStopIntake");
    private final DoubleLogger log_autonActionTimes = new DoubleLogger(AutonK.kLogTab, "autonActionTimes");

    // logic booleans
    private boolean m_isAtStopShoot = false;
    private boolean m_isAtStopIntake = false;
    private Trigger trg_isAtStopShoot = new Trigger(() -> m_isAtStopShoot);
    private Trigger trg_isAtStopIntake = new Trigger(() -> m_isAtStopIntake);

    public WaltAdaptableAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake, Shooter shooter, Swerve swerve) {
        m_superstructure = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
        m_shooter = shooter;
        m_drivetrain = swerve;
    }

    //---UTILITY METHODS
    // private Command logState(double state) {
    //     return Commands.runOnce(() -> log_autonState.accept(state));
    // }

    private Command tp(String message) {
        return WaltLogger.timedPrintCmd(message);
    }

    private Command waitIntakeHomedCmd() {
        return Commands.waitUntil(m_intake.intakeHomedSupp);
    }

    private Command homingCmd() {
        return Commands.sequence(tp("intakeArmHoming.START"), waitIntakeHomedCmd(), tp("intakeArmHoming.END"))
            .withTimeout(5);
    }

    public AutoRoutine preheater() {
        System.out.println("PREHEAT MADE");
        return adaptableAuton("PreHeat", new AdaptableAutonInfo("PreHeat", AutonK.kShootingTimeout, false, 0));
    }

    /**
     * Eagerly loads every given trajectory into the AutoFactory's TrajectoryCache.
     * After this runs, {@code routine.trajectory(name)} during autonomousInit is a
     * HashMap hit instead of a disk read + GSON parse. Call once during robotInit.
     */
    public void preloadAllTrajectories(String[] names) {
        var cache = m_autoFactory.cache();
        long totalStart = System.nanoTime();
        for (String name : names) {
            long ts = System.nanoTime();
            cache.loadTrajectory(name);
            long elapsed = System.nanoTime() - ts;
            if (elapsed > 5_000_000) { // only log if > 5ms
                System.out.printf("[PRELOAD] %s: %.1f ms%n", name, elapsed * 1e-6);
            }
        }
        System.out.printf("[PRELOAD] %d trajectories total: %.1f ms%n",
            names.length, (System.nanoTime() - totalStart) * 1e-6);
    }

    //thank you grac
    private static Command printLater(Supplier<String> stringSup) {
		return Commands.defer(() -> {
			return Commands.print(stringSup.get());
		}, Set.of());
	}

    public Command logTimer(String epochName, Supplier<Timer> timerSup) {
        return printLater(() -> {
            var timer = timerSup.get();
            log_autonActionTimes.accept(autonTimer.get());
            return epochName + " at " + timer.get() + " s";
        });
	}

    public void startAutonTimer() {
        autonTimer.start();
    }

    //---AUTOROUTINE TRAJECTORY HELPER
    private AutoTrajectory createTraj(AutoRoutine routine, String name) {
        AutoTrajectory traj = routine.trajectory(name);
        // Go through the factory's TrajectoryCache so this is a HashMap hit after preload,
        // instead of a second disk read + GSON parse on top of routine.trajectory(name)'s load.
        var poses = m_autoFactory.cache().loadTrajectory(name).get().getPoses();
        traj.active().onTrue(Commands.sequence(
            Commands.runOnce(() -> {
                log_trajectoryName.accept(name);
                log_trajectoryPoses.accept(poses);
            }),
            tp("traj.START(" + name + ")")
        ));
        traj.done().onTrue(tp("traj.END(" + name + ")"));
        return traj;
    }

    //---ADAPTABLE AUTON MAKER
    public AutoRoutine adaptableAuton(String routineName, AdaptableAutonInfo autonInfo) {
        AutoRoutine routine = m_autoFactory.newRoutine(routineName);

        String path = autonInfo.autonName();
        AutoTrajectory traj = createTraj(routine, path);

        routine.active().onTrue(
            traj.cmd().alongWith(homingCmd())
        );

        setUpTrajTriggers(traj, autonInfo.shooterTimeout(), autonInfo.SOTM());

        return routine;
    }

    public AutoRoutine multiAdaptableAuton(String routineName, AdaptableAutonInfo[] autonInfos) {
        System.out.println("================== adaptableBuilder Start ==================");

        AutoTrajectory[] autonTrajs = new AutoTrajectory[autonInfos.length];
        AutoRoutine routine = m_autoFactory.newRoutine(routineName);

        for (int i = 0; i < autonInfos.length; i++) {
            String path = autonInfos[i].autonName();
            autonTrajs[i] = createTraj(routine, path);
        }
        System.out.println("trajs built");


        for (int i = 0; i < autonTrajs.length; i++) {
            AutoTrajectory thisTraj = autonTrajs[i];
            var thisInfo = autonInfos[i];
            System.out.println("traj idx " + i + " (" + thisInfo.autonName + ") .done().onTrue() built");

            setUpTrajTriggers(thisTraj, thisInfo.shooterTimeout(), autonInfos[i].SOTM());
        }
        System.out.println("trajTriggers built");

        routine.active().onTrue(
            Commands.sequence(
                Commands.waitSeconds(autonInfos[0].delay()),
                autonTrajs[0].cmd().alongWith(homingCmd())
            )
        );

        System.out.println("routine active built");


        System.out.println("traj idx onTrue pre-built");
        for (int i = 0; i < autonTrajs.length - 1; i++) {
            AutoTrajectory thisTraj = autonTrajs[i];
            var thisInfo = autonInfos[i];
            System.out.println("traj idx " + i + " (" + thisInfo.autonName + ") .done().onTrue() built");
            // thisTraj.active().whileTrue(
            //     Commands.run(() -> {
            //         var sample = thisTraj.getRawTrajectory().sampleAt(i, m_isAtStopIntake);
            //     }))
            // )
            if (autonInfos[i + 1].delay() == 0) {
                thisTraj.done().onTrue(
                thisInfo.SOTM ? Commands.sequence(
                    Commands.waitSeconds(autonInfos[i + 1].delay()),
                    autonTrajs[i + 1].cmd()) : 
                    Commands.sequence(
                        tp("WAITING FOR SHOOTING DONE"),
                        Commands.race(
                            Commands.waitUntil(m_shooter.getBallShotDebounceTrg().or(trg_isAtStopShoot)),
                            Commands.waitSeconds(thisInfo.shooterTimeout())
                        ),
                        tp("TRAJ " + (i + 1) + " DELAY STARTED"),
                        Commands.waitSeconds(autonInfos[i + 1].delay()),
                        tp("TRAJ " + (i + 1) + " STARTED"),
                        autonTrajs[i + 1].cmd()
                    ));
            } else {
                thisTraj.done().onTrue(
                thisInfo.SOTM ? autonTrajs[i + 1].cmd() : 
                    Commands.sequence(
                        tp("WAITING FOR SHOOTING DONE"),
                        Commands.race(
                            Commands.waitUntil(m_shooter.getBallShotDebounceTrg().or(trg_isAtStopShoot)),
                            Commands.waitSeconds(thisInfo.shooterTimeout())
                        ),
                        tp("TRAJ " + (i + 1) + " STARTED"),
                        autonTrajs[i + 1].cmd()
                    ));
            }
        }

        autonTrajs[autonTrajs.length - 1].done().onTrue(
            m_drivetrain.xBrakeCmd()
        );

        System.out.println("================== full routine built ==================");

        return routine;
    }

    private StringLogger log_autonEventMarker = WaltLogger.logString("Auton", "autonTriggerCall");

    public Command updateAutonEventMarkerLogger(String update) {
        return Commands.runOnce(() -> log_autonEventMarker.accept(update));
    }

    /* CHECK IF TURRET IS ABLE TO SHOOT FOR PASSING PLEASEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
    public void setUpTrajTriggers(AutoTrajectory traj, double shooterTimeout, boolean SOTM) {
        //---TRIGGER ACTIONS
        traj.atTime(kIntakeWaypoint).onTrue(
            m_superstructure.intake(() -> false, () -> false).until(trg_isAtStopIntake)
        );

        // traj.atTime(kIntakeWaypoint).onTrue(
        //     logTimer("intaking", () -> autonTimer)
        // );

        traj.atTime(kShootWaypoint).and(() -> !SOTM).onTrue(
            // stopShoot acts as an override STOP SHOOTING to continue the pathing whereas the debounceTrg can help us move on faster if we're already outta balls
            Commands.race(
                m_superstructure.activateOuttakeShotCalc().until(m_shooter.getBallShotDebounceTrg()),
                Commands.waitSeconds(shooterTimeout)
            )
            // (m_superstructure.activateOuttakeShotCalc().until(m_shooter.getBallShotDebounceTrg())).withTimeout(shooterTimeout)
        );

        traj.atTime(kShootWaypoint).and(() -> SOTM).onTrue(
            // stopShoot acts as an override STOP SHOOTING to continue the pathing whereas the debounceTrg can help us move on faster if we're already outta balls
            m_superstructure.activateOuttakeShotCalc().until(m_shooter.getBallShotDebounceTrg().or(trg_isAtStopShoot))
        );

        traj.atTime(kShootWaypoint).onTrue(
            m_superstructure.intakeShimmy(() -> true)
        );

        // traj.atTime(kShootWaypoint).onTrue(
        //     logTimer(kShootWaypoint, () -> autonTimer)
        // );

        traj.atTime(kStopShootWaypoint).onChange(
            Commands.runOnce(() -> {
                m_isAtStopShoot = !m_isAtStopShoot;
                log_isAtStopShoot.accept(m_isAtStopShoot);
            })
        );

        traj.atTime(kStopIntakeWaypoint).onTrue(
            Commands.runOnce(() -> {
                m_isAtStopIntake = true;
                log_isAtStopIntake.accept(m_isAtStopIntake);
            })
        );

        traj.atTime(kStopIntakeWaypoint).onFalse(
            Commands.runOnce(() -> {
                m_isAtStopIntake = false;
                log_isAtStopIntake.accept(m_isAtStopIntake);
            })
        );

        traj.atTime(kIntakeAndShootWaypoint).onTrue(
            m_superstructure.intake(() -> true, () -> false).until(traj.atTime(kStopIntakeAndShootWaypoint))
        );

        traj.atTime(kIntakeAndShootWaypoint).onTrue(
            m_superstructure.activateOuttakeShotCalc().until(traj.atTime(kStopIntakeAndShootWaypoint))
        );

        //---TRIGGER LOGGERS
        traj.atTime(kIntakeWaypoint).onTrue(
            updateAutonEventMarkerLogger(kIntakeWaypoint)
        );

        traj.atTime(kIntakeAndShootWaypoint).onTrue(
            updateAutonEventMarkerLogger(kIntakeAndShootWaypoint)
        );

        traj.atTime(kStopIntakeAndShootWaypoint).onTrue(
            updateAutonEventMarkerLogger(kStopIntakeAndShootWaypoint)
        );

        traj.atTime(kStopIntakeWaypoint).onTrue(
            updateAutonEventMarkerLogger(kStopIntakeWaypoint)
        );

        traj.atTime(kShootWaypoint).onTrue(
            updateAutonEventMarkerLogger(kShootWaypoint)
        );

        traj.atTime(kStopShootWaypoint).onTrue(
            updateAutonEventMarkerLogger(kStopShootWaypoint)
        );
    }
    
    public final record AdaptableAutonInfo(
        String autonName,
        double shooterTimeout,
        boolean SOTM,
        double delay
    ) {}
};