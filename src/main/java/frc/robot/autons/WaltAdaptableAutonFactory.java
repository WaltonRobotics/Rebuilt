package frc.robot.autons;

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

/*
 * WaltAdaptableAutonFactory
 *
 * This class is responsible for building all of our autonomous routines.
 * Rather than writing a brand new command sequence for every single auton we
 * want to run, this factory takes a list of trajectory segments + settings and
 * assembles the full routine automatically — hence "adaptable".
 *
 * Quick vocabulary for new programmers:
 *   Trajectory  — a pre-planned path the robot drives along, created in the
 *                 Choreo desktop app and saved as a JSON file on the robot.
 *   Event marker / waypoint — a named timestamp placed inside Choreo.
 *                 When the robot's path timer hits that timestamp, a Trigger
 *                 fires and we run a command (e.g. start intaking, start shooting).
 *   Trigger     — a WPILib class that watches a boolean condition and runs
 *                 commands when that condition becomes true or false.
 *   SOTM        — "Shoot On The Move". Instead of stopping the robot to shoot,
 *                 SOTM mode lets the robot keep driving while the shooter is active.
 *
 * The two main entry points are:
 *   adaptableAuton(...)       — builds a routine that follows a single path
 *   multiAdaptableAuton(...)  — builds a routine that chains multiple paths together
 *
 * AutonChooser.java is where the actual routines are defined (which paths to use,
 * what timeouts, SOTM or not, etc.) and put on the dashboard for the driver to pick.
 */
public class WaltAdaptableAutonFactory {

    // ---- SUBSYSTEMS ----
    // These are passed in through the constructor so we can tell them what to do
    // without this class being responsible for creating them.

    private final Superstructure m_superstructure; // controls shooting + intaking at a high level
    private final AutoFactory    m_autoFactory;    // Choreo: loads trajectory files and creates routines
    private final Intake         m_intake;         // intake arm + rollers
    private final Shooter        m_shooter;        // flywheels + shot calculator
    private final Swerve         m_drivetrain;     // swerve drive (used to lock wheels at the end)

    // ---- WAYPOINT NAMES ----
    // These strings must EXACTLY match the event marker names you place in Choreo.
    // If a marker is named "intake" in Choreo, it has to be "intake" here too.

    private static final String kIntakeWaypoint = "intake";
    private static final String kStopIntakeWaypoint = "stopIntake";
    private static final String kShootWaypoint = "shoot";
    private static final String kStopShootWaypoint = "stopShoot";
    private static final String kIntakeAndShootWaypoint = "intakeAndShoot";
    private static final String kStopIntakeAndShootWaypoint = "stopIntakeAndShoot";

    // ---- loggers ----
    // WaltLogger pushes data to NetworkTables, which AdvantageScope can read
    // and replay after a match so we can see exactly what happened and when.

    private final StringLogger log_trajectoryName   = new StringLogger(AutonK.kLogTab, "trajectoryName");
    private final Pose2dArrayLogger log_trajectoryPoses  = new Pose2dArrayLogger(AutonK.kLogTab, "trajectoryPoses");
    private final BooleanLogger log_isAtStopShoot    = new BooleanLogger(AutonK.kLogTab, "isAtStopShoot");
    private final BooleanLogger log_isAtStopIntake   = new BooleanLogger(AutonK.kLogTab, "isAtStopIntake");
    private final DoubleLogger log_autonActionTimes = new DoubleLogger(AutonK.kLogTab, "autonActionTimes");
    private final StringLogger log_autonEventMarker = WaltLogger.logString("Auton", "autonTriggerCall");

    // ---- state flags ----
    // These get flipped by trajectory event markers at runtime and are exposed
    // as Triggers so we can use them as end conditions for running commands.

    // Toggled by the "stopShoot" marker. Acts as a manual override to end shooting
    // early — useful when the robot has driven past the point where shooting makes sense
    // but the ball sensor hasn't confirmed the shot yet.
    // Uses onChange (fires on both the rising and falling edge of the marker window)
    // so the flag automatically resets itself once the window closes.
    private boolean m_isAtStopShoot = false;

    // Goes true while the robot is inside the "stopIntake" marker window, then
    // back to false when it leaves. The intake command runs until this goes true.
    private boolean m_isAtStopIntake = false;

    private final Trigger trg_isAtStopShoot = new Trigger(() -> m_isAtStopShoot);
    private final Trigger trg_isAtStopIntake = new Trigger(() -> m_isAtStopIntake);

    // ---- auton timer ----
    // Tracks time since auton started. Used to timestamp logged events so we
    // know exactly when each action fired during a match.

    public Timer autonTimer = new Timer();


    // =============================================================
    // CONSTRUCTOR
    // =============================================================

    public WaltAdaptableAutonFactory(
            Superstructure superstructure,
            AutoFactory autoFactory,
            Intake intake,
            Shooter shooter,
            Swerve swerve) {
        m_superstructure = superstructure;
        m_autoFactory    = autoFactory;
        m_intake         = intake;
        m_shooter        = shooter;
        m_drivetrain     = swerve;
    }


    // =============================================================
    // PRIVATE UTILITY METHODS
    // =============================================================

    // Short helper to print a timestamped message when a command runs.
    // Used throughout to trace the auton flow in the console/logs.
    private Command timedPrint(String message) {
        return WaltLogger.timedPrintCmd(message);
    }

    // Defers evaluating the string until the command actually *runs*, not when
    // it's constructed. This matters for anything that reads a live value like
    // a timer — without defer, the value would be captured at build time (t=0).
    // Thank you grac
    private static Command printLater(Supplier<String> stringSup) {
        return Commands.defer(() -> Commands.print(stringSup.get()), Set.of());
    }

    // Waits for the intake arm to find its home position (mechanical zero)
    // before the auton relies on any intake movement. Times out after 5 s so
    // a bad homing wont screw our path over
    private Command homingCmd() {
        return Commands.sequence(
            timedPrint("intakeArmHoming.START"),
            Commands.waitUntil(m_intake.intakeHomedSupp),
            timedPrint("intakeArmHoming.END")
        ).withTimeout(5);
    }

    // Builds the command that fires when trajectory[i] finishes and trajectory[i+1] should start.
    //
    // SOTM mode   → drive into the next path right away (optionally after a short delay).
    //               Shooting is already happening while driving, so no wait needed.
    //
    // Normal mode → wait for the shooter to confirm the ball has left the robot
    //               (or for the stopShoot override, or until the timeout expires),
    //               then optionally delay, then start the next path.
    private Command buildTransitionCmd(
            Command nextTrajCmd,
            double nextDelay,
            double shooterTimeout,
            boolean shootOnTheMove) {

        // If there's a pre-path delay, prepend it to whatever we're about to start.
        Command startNext = nextDelay > 0
            ? Commands.sequence(Commands.waitSeconds(nextDelay), nextTrajCmd)
            : nextTrajCmd;

        if (shootOnTheMove) {
            return startNext; // SOTM — just go, no waiting needed
        }

        // Non-SOTM: race shot confirmation against the timeout so we never
        // get stuck waiting if a ball gets stuck or the sensor misses.
        Command waitForShot = Commands.race(
            Commands.waitUntil(m_shooter.getBallShotDebounceTrg().or(trg_isAtStopShoot)),
            Commands.waitSeconds(shooterTimeout)
        );

        return Commands.sequence(
            timedPrint("WAITING FOR SHOOTING DONE"),
            waitForShot,
            startNext
        );
    }

    // Logs a named waypoint event to NetworkTables so post-match replays in
    // AdvantageScope show which markers fired and in what order.
    private Command logEventMarker(String markerName) {
        return Commands.runOnce(() -> log_autonEventMarker.accept(markerName));
    }


    // =============================================================
    // PUBLIC UTILITIES
    // =============================================================

    // Eagerly parses and caches every trajectory during robotInit.
    // Without this, the first time a trajectory is requested during auton it
    // has to be read off the filesystem + parsed from JSON, which takes time.
    // After preloading, all lookups are fast HashMap hits instead.
    // Call this once during robotInit, before building any auton routines.
    public void preloadAllTrajectories(String[] names) {
        var cache = m_autoFactory.cache();
        long totalStart = System.nanoTime();

        for (String name : names) {
            long ts = System.nanoTime();
            cache.loadTrajectory(name);
            long elapsed = System.nanoTime() - ts;

            // only print slow (> 5 ms) crine because rio cant handle too many prints
            if (elapsed > 5_000_000) {
                System.out.printf("[PRELOAD] %s: %.1f ms%n", name, elapsed * 1e-6);
            }
        }

        System.out.printf("[PRELOAD] %d trajectories total: %.1f ms%n",
            names.length, (System.nanoTime() - totalStart) * 1e-6);
    }

    // Logs an elapsed-time snapshot for a named event.
    // Used to timestamp when specific things happened during auton for post-match analysis.
    public Command logTimer(String epochName, Supplier<Timer> timerSup) {
        return printLater(() -> {
            var timer = timerSup.get();
            log_autonActionTimes.accept(autonTimer.get());
            return epochName + " at " + timer.get() + " s";
        });
    }

    // Start the auton-wide timer. Call this at the top of autonomousInit.
    public void startAutonTimer() {
        autonTimer.start();
    }


    // =============================================================
    // TRAJECTORY BUILDER HELPER
    // =============================================================

    // Creates an AutoTrajectory and wires up start/end logging.
    // Poses are pulled from the cache (populated by preloadAllTrajectories)
    // so we're doing a fast lookup, not re-parsing the JSON file from disk.
    private AutoTrajectory createTraj(AutoRoutine routine, String name) {
        AutoTrajectory traj = routine.trajectory(name);
        var poses = m_autoFactory.cache().loadTrajectory(name).get().getPoses();

        traj.active().onTrue(Commands.sequence(
            Commands.runOnce(() -> {
                log_trajectoryName.accept(name);
                log_trajectoryPoses.accept(poses);
            }),
            timedPrint("traj.START(" + name + ")")
        ));

        traj.done().onTrue(timedPrint("traj.END(" + name + ")"));

        return traj;
    }


    // =============================================================
    // AUTON ROUTINE BUILDERS
    // =============================================================

    // Builds a "preheat" routine that runs while the robot is still disabled
    // before a match. Ensures we dont stall for like 0.4 seconds at the start of auton
    public AutoRoutine preheater() {
        System.out.println("PREHEAT MADE");
        return adaptableAuton("PreHeat", new AdaptableAutonInfo("MISC/PreHeat", AutonK.kShootingTimeout, false, 0));
    }

    // Builds a single-segment auton: follow one trajectory and fire waypoint
    // actions at the event markers embedded in it.
    // The intake arm homing sequence runs alongside the path so homing doesn't
    // cost us any auton time.
    public AutoRoutine adaptableAuton(String routineName, AdaptableAutonInfo autonInfo) {
        AutoRoutine routine = m_autoFactory.newRoutine(routineName);
        AutoTrajectory traj = createTraj(routine, autonInfo.autonName());

        routine.active().onTrue(
            traj.cmd().alongWith(homingCmd())
        );

        setUpTrajTriggers(traj, autonInfo.shooterTimeout(), autonInfo.SOTM());

        return routine;
    }

    // Builds a multi-segment auton by chaining several trajectories end-to-end.
    // How the transition between segments works depends on SOTM mode — see buildTransitionCmd.
    // After the last segment finishes, the drivetrain locks into an X-brake so the robot
    // doesn't slide.
    /**
     * NEW TERMINOLOGY: COAST OUT -- Coasting out means to let the motors keep their momentum when they stop, instead of 
     * simply going to zero (stopping abruptly).
     */
    // Something to think about is to coast out the swerve at the end of auto, to cover more ground to get closer to
    // fuel to pick up faster?
    // EX: See Citrus's (1678) autos
    public AutoRoutine multiAdaptableAuton(String routineName, AdaptableAutonInfo[] autonInfos) {
        AutoRoutine routine = m_autoFactory.newRoutine(routineName);

        // Step 1: create all trajectory objects before wiring any triggers.
        // This ensures every trajectory is in memory before anything tries to
        // reference the next one.
        AutoTrajectory[] autonTrajs = new AutoTrajectory[autonInfos.length];
        for (int i = 0; i < autonInfos.length; i++) {
            autonTrajs[i] = createTraj(routine, autonInfos[i].autonName());
        }

        // Step 2: attach waypoint triggers to every segment.
        for (int i = 0; i < autonTrajs.length; i++) {
            setUpTrajTriggers(autonTrajs[i], autonInfos[i].shooterTimeout(), autonInfos[i].SOTM());
        }

        // Step 3: start the first trajectory when the routine becomes active.
        // Home the intake arm in parallel so we don't waste time waiting for it.
        Command firstPath  = autonTrajs[0].cmd().alongWith(homingCmd());
        double  firstDelay = autonInfos[0].delay();
        routine.active().onTrue(
            firstDelay > 0
                ? Commands.sequence(Commands.waitSeconds(firstDelay), firstPath)
                : firstPath
        );

        // Step 4: chain each segment into the next.
        // For every segment except the last: "when this path ends, run the transition into the next one."
        for (int i = 0; i < autonTrajs.length - 1; i++) {
            Command transition = buildTransitionCmd(
                autonTrajs[i + 1].cmd(),
                autonInfos[i + 1].delay(),
                autonInfos[i].shooterTimeout(),
                autonInfos[i].SOTM()
            );
            autonTrajs[i].done().onTrue(transition);
        }

        // Step 5: lock wheels once the final segment finishes.
        autonTrajs[autonTrajs.length - 1].done().onTrue(
            m_drivetrain.xBrakeCmd()
        );

        return routine;
    }


    // =============================================================
    // TRAJECTORY TRIGGER SETUP
    // =============================================================

    // Registers all the waypoint-triggered commands for a single trajectory segment.
    //
    // Choreo fires a Trigger each time the path timer reaches a named event marker.
    // That's how the robot knows *when* to intake, shoot, etc. based on where it is
    // along the path — we're not polling position, just reacting to timestamps.
    //
    // Supported marker names (type these exactly in the Choreo editor):
    //   "intake"              start intaking; runs until stopIntake fires
    //   "stopIntake"          stop the active intake command
    //   "shoot"               start shooting (behavior differs by SOTM flag, see below)
    //   "stopShoot"           force-stop shooting early (override if ball sensor misses)
    //   "intakeAndShoot"      run intake + shooter at the same time
    //   "stopIntakeAndShoot"  stop the simultaneous intake+shoot
    //
    // shootOnTheMove (SOTM):
    //   false → robot waits for shot confirmation before moving on (timeout is the safety cutoff)
    //   true  → robot keeps driving while shooting; shooting ends when ball exits or stopShoot fires
    public void setUpTrajTriggers(AutoTrajectory traj, double shooterTimeout, boolean shootOnTheMove) {

        // "intake" — start intaking, stop when the stopIntake marker fires
        traj.atTime(kIntakeWaypoint).onTrue(
            m_superstructure.intake(() -> false, () -> false).until(trg_isAtStopIntake)
        );

        // "shoot" (non-SOTM) — race: shoot until ball exits OR timeout expires.
        // The timeout is a safety cutoff so auton never stalls if a ball gets stuck.
        traj.atTime(kShootWaypoint).and(() -> !shootOnTheMove).onTrue(
            Commands.race(
                m_superstructure.activateOuttakeShotCalc().until(m_shooter.getBallShotDebounceTrg()),
                Commands.waitSeconds(shooterTimeout)
            )
        );

        // "shoot" (SOTM) — robot keeps driving; shooting ends when ball exits or stopShoot fires
        traj.atTime(kShootWaypoint).and(() -> shootOnTheMove).onTrue(
            m_superstructure.activateOuttakeShotCalc()
                .until(m_shooter.getBallShotDebounceTrg().or(trg_isAtStopShoot))
        );

        // "shoot" (both modes) — shimmy the intake to help feed the ball into the shooter
        traj.atTime(kShootWaypoint).onTrue(
            m_superstructure.intakeShimmy(() -> true)
        );

        // "stopShoot" — toggle the override flag on both the rising AND falling
        // edge of the marker window (onChange fires twice per pass-through).
        // This means the flag flips on as the robot enters the window, then
        // automatically flips back off as it leaves — a self-resetting override.
        traj.atTime(kStopShootWaypoint).onChange(
            Commands.runOnce(() -> {
                m_isAtStopShoot = !m_isAtStopShoot;
                log_isAtStopShoot.accept(m_isAtStopShoot);
            })
        );

        // "stopIntake" — true while inside the marker window, false once outside.
        // The intake command above uses trg_isAtStopIntake as its end condition.
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

        // "intakeAndShoot" / "stopIntakeAndShoot" — run intake + shooter together
        // until the stop marker fires. Used for bump/passing actions.
        traj.atTime(kIntakeAndShootWaypoint).onTrue(
            m_superstructure.intake(() -> true, () -> false)
                .until(traj.atTime(kStopIntakeAndShootWaypoint))
        );
        traj.atTime(kIntakeAndShootWaypoint).onTrue(
            m_superstructure.activateOuttakeShotCalc()
                .until(traj.atTime(kStopIntakeAndShootWaypoint))
        );

        // Log every marker event so post-match AdvantageScope replays show
        // which events fired and in what order.
        traj.atTime(kIntakeWaypoint).onTrue(logEventMarker(kIntakeWaypoint));
        traj.atTime(kStopIntakeWaypoint).onTrue(logEventMarker(kStopIntakeWaypoint));
        traj.atTime(kShootWaypoint).onTrue(logEventMarker(kShootWaypoint));
        traj.atTime(kStopShootWaypoint).onTrue(logEventMarker(kStopShootWaypoint));
        traj.atTime(kIntakeAndShootWaypoint).onTrue(logEventMarker(kIntakeAndShootWaypoint));
        traj.atTime(kStopIntakeAndShootWaypoint).onTrue(logEventMarker(kStopIntakeAndShootWaypoint));
    }


    // =============================================================
    // DATA RECORD
    // =============================================================

    /*
     * AdaptableAutonInfo
     *
     * Holds all the settings for one trajectory segment of an auton routine.
     * Think of it as the "config card" you hand to the factory for each path.
     *
     *   autonName      — file name of the Choreo trajectory (no .traj extension)
     *   shooterTimeout — max seconds to wait for shot confirmation before giving up
     *                    and moving on (only matters in non-SOTM mode)
     *   SOTM           — Shoot On The Move: true = shoot while driving,
     *                    false = stop and wait for shot confirmation
     *   delay          — seconds to wait before starting this segment (0 = start right away).
     *                    Useful when following another robot or letting defense clear out.
     */
    public final record AdaptableAutonInfo(
        String  autonName,
        double  shooterTimeout,
        boolean SOTM,
        double  delay
    ) {}
}
