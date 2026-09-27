package frc.robot.dashboards;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

import org.wpilib.tunable.Tunables;

import choreo.auto.AutoChooser;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.autons.WaltAdaptableAutonFactory;
import frc.robot.autons.WaltAdaptableAutonFactory.AdaptableAutonInfo;

public class AutonChooser {
    public static AutoChooser m_chooser;
    public static WaltAdaptableAutonFactory m_adaptableAutonFactory;

    // Registry of every auton registered with the chooser. Walked during warmup to
    // pre-build routines and to extract the set of trajectory files to preload.
    private record AutonEntry(String name, AdaptableAutonInfo infos) {}
    private static final List<AutonEntry> s_autons = new ArrayList<>();
    private record MultiAutonEntry(String name, AdaptableAutonInfo[] infos) {}
    private static final List<MultiAutonEntry> s_multiAutons = new ArrayList<>();

    /* AUTON NAMES */
    //---2 CYCLES
    private final static String kRightTrenchTwoCycleBumpReturn = "RIGHT Trench 2 Cycle Bump Return";
    private final static String kLeftTrenchTwoCycleBumpReturn = "LEFT Trench 2 Cycle Bump Return";

    //---FAST 2 CYCLES
    private final static String kRightTrenchTwoCycleBumpReturnFast = "RIGHT Trench 2 Cycle Bump Return Fast";
    private final static String kLeftTrenchTwoCycleBumpReturnFast = "LEFT Trench 2 Cycle Bump Return Fast";
    private final static String kRightTrechTwoCycleTrenchReturnFast = "RIGHT Trench 2 Cycle Trench Return Fast";
    private final static String kLeftTrenchTwoCycleTrenchReturnFast = "LEFT Trench 2 Cycle Trench Return Fast";
    // fast is defined as returning to the NZ at the end of auton

    //---2 CYCLES PLUS DEPOT
    private final static String kLeftTrenchTwoCycleBumpReturnDepot = "LEFT Trench 2 Cycle Bump Return Plus Depot";
    private final static String kLeftTrenchTwoCycleTrenchReturnDepot = "LEFT Trench 2 Cycle Trench Return Plus Depot";

    //--MISC
    private final static String kRightTrenchSelfPass = "RIGHT Orbit";
    private final static String kLeftTrenchSelfPass = "LEFT Orbit";

    public static void initialize(WaltAdaptableAutonFactory adaptableAutonFactory) {
        m_adaptableAutonFactory = adaptableAutonFactory;
        m_chooser = new AutoChooser();
        s_multiAutons.clear();

        /* AUTON OPTIONS */
        //---2 CYCLES
        addMultiAuton(kRightTrenchTwoCycleBumpReturn,
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoGoOut, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kLeftTrenchTwoCycleBumpReturn,
            new AdaptableAutonInfo(AutonK.kLeftOneBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kRightTrenchTwoCycleBumpReturnFast,
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturnFast, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrenchFast, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpReturnFast, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrenchFast, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoGoOut, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kLeftTrenchTwoCycleBumpReturnFast,
            new AdaptableAutonInfo(AutonK.kLeftOneBumpReturnFast, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrenchFast, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturnFast, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrenchFast, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kRightTrechTwoCycleTrenchReturnFast,
            new AdaptableAutonInfo(AutonK.kRightOneTrenchReturn, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoTrenchReturn, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoGoOut, AutonK.kSOTMTimeout, false, 0));

        addMultiAuton(kLeftTrenchTwoCycleTrenchReturnFast,
            new AdaptableAutonInfo(AutonK.kLeftOneTrenchReturn, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchReturn, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, AutonK.kSOTMTimeout, false, 0));

        //---2 CYCLES PLUS DEPOT
        addMultiAuton(kLeftTrenchTwoCycleBumpReturnDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToDepot, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepotSweep, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepotToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kLeftTrenchTwoCycleTrenchReturnDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneTrenchReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchToDepot, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepotSweep, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepotToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchReturn, AutonK.kSOTMTimeout, false, 0));

        //---MISC
        addAuton(kRightTrenchSelfPass, new AdaptableAutonInfo(AutonK.kRightOneSelfPass, AutonK.kSOTMTimeout, true, 0));
        addAuton(kLeftTrenchSelfPass, new AdaptableAutonInfo(AutonK.kLeftOneSelfPass, AutonK.kSOTMTimeout, true, 0));
        
        // Load AutonChooser
        Tunables.publish("AutoChooser", m_chooser);
    }

    /**
     * Registers an auton with the chooser and records it in the warmup registry.
     * Captures {@code info} once per auton rather than re-allocating the array
     * on every auto start like the previous inline lambdas did.
     */
    private static void addAuton(String name, AdaptableAutonInfo info) {
        s_autons.add(new AutonEntry(name, info));
        m_chooser.addRoutine(name,
            () -> m_adaptableAutonFactory.adaptableAuton(name, info));
    }

    /**
     * Registers an auton with the chooser and records it in the warmup registry.
     * Captures {@code infos} once per auton rather than re-allocating the array
     * on every auto start like the previous inline lambdas did.
     */
    private static void addMultiAuton(String name, AdaptableAutonInfo... infos) {
        s_multiAutons.add(new MultiAutonEntry(name, infos));
        m_chooser.addRoutine(name,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(name, infos));
    }

    /**
     * Returns the unique set of trajectory files referenced by any registered
     * auton, plus the PreHeat trajectory used by the preheater command.
     */
    public static String[] allTrajectoryNames() {
        LinkedHashSet<String> names = new LinkedHashSet<>();
        names.add(AutonK.kPreheatTrajectory);
        for (AutonEntry e : s_autons) {
            names.add(e.infos().autonName());
        }
        for (MultiAutonEntry e : s_multiAutons) {
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
     * the command-composition graphs used by {@code adaptableAuton} and {@code multiAdaptableAuton}.
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
        long totalStart = System.nanoTime();
        int count = 0;
        for (AutonEntry e : s_autons) {
            long ts = System.nanoTime();
            m_adaptableAutonFactory.adaptableAuton(e.name(), e.infos());
            count++;
            System.out.printf("[PREHEAT] %s: %.1f ms%n", e.name(), (System.nanoTime() - ts) * 1e-6);
        }
        for (MultiAutonEntry e : s_multiAutons) {
            long ts = System.nanoTime();
            m_adaptableAutonFactory.multiAdaptableAuton(e.name(), e.infos());
            count++;
            System.out.printf("[PREHEAT] %s: %.1f ms%n", e.name(), (System.nanoTime() - ts) * 1e-6);
        }
        System.out.printf("[PREHEAT] %d routines total: %.1f ms%n",
            count, (System.nanoTime() - totalStart) * 1e-6);
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
        long totalStart = System.nanoTime();
        for (String cls : classes) {
            try {
                long ts = System.nanoTime();
                Class.forName(cls, true, cl);
                long elapsed = System.nanoTime() - ts;
                if (elapsed > 5_000_000) { // only log if > 5ms
                    System.out.printf("[CLASSLOAD] %s: %.1f ms%n", cls, elapsed * 1e-6);
                }
            } catch (ClassNotFoundException e) {
                DriverStationErrors.reportWarning(
                    "ChoreoLib warmup: class not found: " + cls + " (library version mismatch?)",
                    false);
            }
        }
        System.out.printf("[CLASSLOAD] %d classes total: %.1f ms%n",
            classes.length, (System.nanoTime() - totalStart) * 1e-6);
    }

    public static Command getPreheater() {
        if (m_adaptableAutonFactory == null) {
            return Commands.print("Tried to preheat before factory init!");
        }

        // return m_adaptableAutonFactory.preheater().cmd().ignoringDisable(true);
        return m_chooser.selectedCommand();
    }
}
