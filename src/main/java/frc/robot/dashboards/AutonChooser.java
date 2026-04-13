package frc.robot.dashboards;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.autons.WaltAdaptableAutonFactory;
import frc.robot.autons.WaltAdaptableAutonFactory.AdaptableAutonInfo;

public class AutonChooser {
    private static final String kPreheatTrajectory = "PreHeat";

    public static AutoChooser m_chooser;
    public static WaltAdaptableAutonFactory m_adaptableAutonFactory;

    // Registry of every auton registered with the chooser. Walked during warmup to
    // pre-build routines and to extract the set of trajectory files to preload.
    private record AutonEntry(String name, AdaptableAutonInfo[] infos) {}
    private static final List<AutonEntry> s_autons = new ArrayList<>();

    /* AUTON NAMES */
    //---1.5 CYCLES
    private final static String kLeftShootAndSweep = "LEFT Sweep 1.5 Cycle";
    private final static String kRightShootAndSweep = "RIGHT Sweep 1.5 Cycle";
    private final static String kLeftShootAndPass = "LEFT Pass 1.5 Cycle";
    private final static String kRightShootAndPass = "RIGHT Pass 1.5 Cycle";
    private final static String kRightShootAndDepot = "RIGHT Depot 1.5 Cycle";
    private final static String kLeftShootAndDepot = "LEFT Depot 1.5 Cycle";

    //---2 CYCLES
    private final static String kLeftTwoCycle = "LEFT 2 Cycle";
    private final static String kRightTwoCycle = "RIGHT 2 Cycle";
    private final static String kLeftTrenchTwoCycle = "LEFT Trench 2 Cycle";
    private final static String kRightTrenchTwoCycle = "RIGHT Trench 2 Cycle";
    private final static String kLeftDefenseOneCycle = "LEFT Defense";
    private final static String kRightDefenseOneCycle = "RIGHT Defense";
    private final static String kLeftTwoCycleReverse = "LEFT Reverse and Trench-side Jab 2 Cycle";
    private final static String kRightTwoCycleReverse = "RIGHT Reverse and Trench-side Jab 2 Cycle";
    private final static String kLeftTwoCycleReverseAndJab = "LEFT Reverse and Center Jab 2 Cycle";
    private final static String kRightTwoCycleReverseAndJab = "RIGHT Reverse and Center Jab 2 Cycle";

    //---MISC
    private final static String kRightHubCircle = "RIGHT Hub Circle";
    private final static String kLeftSweepAndDepot = "LEFT Sweep and Depot";


    public static void initialize(WaltAdaptableAutonFactory adaptableAutonFactory) {
        m_adaptableAutonFactory = adaptableAutonFactory;
        m_chooser = new AutoChooser();
        s_autons.clear();

        /* AUTON OPTIONS */
        //---MAIN AUTONS
        addAuton(kLeftShootAndSweep,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kSweepShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoSweep, AutonK.kSweepShootingTimeout, true));

        addAuton(kLeftShootAndPass,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoPassing, AutonK.kReshootShootingTimeout, true));

        addAuton(kLeftTwoCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false));

        addAuton(kRightShootAndSweep,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kSweepShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kSweepShootingTimeout, false));

        addAuton(kRightTwoCycle,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false));

        //---TRENCH ASSIST AUTONS
        addAuton(kLeftTrenchTwoCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneTrench, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false));

        addAuton(kRightTrenchTwoCycle,
            new AdaptableAutonInfo(AutonK.kRightOneTrench, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false));

        addAuton(kRightShootAndPass,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kRightTwoPassing, AutonK.kReshootShootingTimeout, true));

        //---DEFENSE AUTONS
        addAuton(kLeftDefenseOneCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneDefense, AutonK.kReshootShootingTimeout, false));

        addAuton(kRightDefenseOneCycle,
            new AdaptableAutonInfo(AutonK.kRightOneDefense, AutonK.kReshootShootingTimeout, false));

        //---DEPOT AUTONS
        addAuton(kLeftShootAndDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepot, AutonK.kReshootShootingTimeout, false));

        addAuton(kRightShootAndDepot,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kRightTwoDepot, AutonK.kReshootShootingTimeout, false));

        //---MISC
        addAuton(kRightHubCircle,
            new AdaptableAutonInfo(AutonK.kRightOneCircle, AutonK.kReshootShootingTimeout, true),
            new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kReshootShootingTimeout, true));

        addAuton(kLeftSweepAndDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneSweepAndDepot, AutonK.kReshootShootingTimeout, true),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, true));

        addAuton(kLeftTwoCycleReverse,
            new AdaptableAutonInfo(AutonK.kLeftOneReverse, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoReverse, AutonK.kReshootShootingTimeout, false));

        addAuton(kRightTwoCycleReverse,
            new AdaptableAutonInfo(AutonK.kRightOneReverse, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kRightTwoReverse, AutonK.kReshootShootingTimeout, false));

        addAuton(kLeftTwoCycleReverseAndJab,
            new AdaptableAutonInfo(AutonK.kLeftOneReverse, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false));

        addAuton(kRightTwoCycleReverseAndJab,
            new AdaptableAutonInfo(AutonK.kRightOneReverse, AutonK.kReshootShootingTimeout, false),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false));

        //Load AutonChooser
        SmartDashboard.putData("AutoChooser", m_chooser);
    }

    /**
     * Registers an auton with the chooser and records it in the warmup registry.
     * Captures {@code infos} once per auton rather than re-allocating the array
     * on every auto start like the previous inline lambdas did.
     */
    private static void addAuton(String name, AdaptableAutonInfo... infos) {
        s_autons.add(new AutonEntry(name, infos));
        m_chooser.addRoutine(name,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(name, infos));
    }

    /**
     * Returns the unique set of trajectory files referenced by any registered
     * auton, plus the PreHeat trajectory used by the preheater command.
     */
    public static String[] allTrajectoryNames() {
        LinkedHashSet<String> names = new LinkedHashSet<>();
        names.add(kPreheatTrajectory);
        for (AutonEntry e : s_autons) {
            for (AdaptableAutonInfo info : e.infos()) {
                names.add(info.autonName());
            }
        }
        return names.toArray(new String[0]);
    }

    /**
     * Builds each registered auto routine once and discards the result. Forces
     * class-loading + early JIT on {@code AutoRoutine}, {@code AutoTrajectory},
     * {@code ChoreoAllianceFlipUtil}, the event-marker trigger machinery, and
     * the command-composition graphs used by {@code multiAdaptableAuton}.
     *
     * <p>Each routine owns a private EventLoop that is only polled when the
     * routine's command is scheduled — since we never schedule these, the bound
     * triggers never fire and the routine objects become GC-eligible immediately.
     *
     * <p>Call AFTER {@link #initialize} and AFTER
     * {@link WaltAdaptableAutonFactory#preloadAllTrajectories} so each routine
     * build is a cache hit instead of a disk read.
     */
    public static void preheatAllRoutines() {
        for (AutonEntry e : s_autons) {
            m_adaptableAutonFactory.multiAdaptableAuton(e.name(), e.infos());
        }
    }

    /**
     * Force-initializes every Choreo class reached on the autonomous hot path.
     * Belt-and-suspenders on top of {@link #preheatAllRoutines} — most of these
     * are loaded transitively once a trajectory is parsed or a routine is built,
     * but an explicit {@code Class.forName} pass eliminates any remaining
     * lazy-init cost that could land in the first autonomousInit tick.
     */
    public static void forceLoadChoreoClasses() {
        String[] classes = {
            // Core
            "choreo.Choreo",
            "choreo.Choreo$TrajectoryCache",
            "choreo.Choreo$TrajectoryLogger",
            // Auto machinery
            "choreo.auto.AutoFactory",
            "choreo.auto.AutoFactory$AllianceContext",
            "choreo.auto.AutoFactory$AutoBindings",
            "choreo.auto.AutoFactory$1",
            "choreo.auto.AutoRoutine",
            "choreo.auto.AutoTrajectory",
            "choreo.auto.AutoTrajectory$1",
            "choreo.auto.AutoTrajectory$2",
            "choreo.auto.AutoChooser",
            // Trajectory model
            "choreo.trajectory.Trajectory",
            "choreo.trajectory.TrajectorySample",
            "choreo.trajectory.SwerveSample",
            "choreo.trajectory.SwerveSample$SwerveSampleStruct",
            "choreo.trajectory.EventMarker",
            "choreo.trajectory.EventMarker$Deserializer",
            // Utilities
            "choreo.util.ChoreoAllianceFlipUtil",
            "choreo.util.ChoreoAllianceFlipUtil$Flipper",
            "choreo.util.ChoreoAllianceFlipUtil$YearInfo",
            "choreo.util.ChoreoAlert",
            "choreo.util.ChoreoAlert$MultiAlert",
            "choreo.util.ChoreoArrayUtil",
            "choreo.util.FieldDimensions",
            "choreo.util.TrajSchemaVersion",
        };
        ClassLoader cl = AutonChooser.class.getClassLoader();
        for (String cls : classes) {
            try {
                Class.forName(cls, true, cl);
            } catch (ClassNotFoundException e) {
                DriverStation.reportWarning(
                    "ChoreoLib warmup: class not found: " + cls + " (library version mismatch?)",
                    false);
            }
        }
    }

    public static Command getPreheater() {
        if (m_adaptableAutonFactory == null) {
            return Commands.print("Tried to preheat before factory init!");
        }

        return m_adaptableAutonFactory.preheater().cmd().ignoringDisable(true);
    }
}
