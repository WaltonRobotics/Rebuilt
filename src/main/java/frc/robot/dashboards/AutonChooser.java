package frc.robot.dashboards;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

import choreo.auto.AutoChooser;
import edu.wpi.first.hal.simulation.AddressableLEDDataJNI;
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
    private record AutonEntry(String name, AdaptableAutonInfo infos) {}
    private static final List<AutonEntry> s_autons = new ArrayList<>();
    private record MultiAutonEntry(String name, AdaptableAutonInfo[] infos) {}
    private static final List<MultiAutonEntry> s_multiAutons = new ArrayList<>();

    /* OLD AUTON NAMES */
    //---1.5 CYCLES
    private final static String kLeftShootAndSweep = "LEFT Sweep 1.5 Cycle";
    private final static String kRightShootAndSweep = "RIGHT Sweep 1.5 Cycle";
    private final static String kLeftShootAndPass = "LEFT Pass 1.5 Cycle";
    private final static String kRightShootAndPass = "RIGHT Pass 1.5 Cycle";
    private final static String kRightShootAndDepot = "RIGHT Depot 1.5 Cycle";
    private final static String kLeftShootAndDepot = "LEFT Depot 1.5 Cycle";

    //---2 CYCLES
    private final static String kLeftTwoCycle = "LEFT Bump 2 Cycle";
    private final static String kRightTwoCycle = "RIGHT Bump 2 Cycle";
    private final static String kLeftTrenchASSISTTwoCycle = "LEFT Bump Trench ASSIST 2 Cycle";
    private final static String kRightTrenchASSISTTwoCycle = "RIGHT Bump Trench ASSIST 2 Cycle";
    private final static String kLeftDefenseOneCycle = "LEFT Bump Defense";
    private final static String kRightDefenseOneCycle = "RIGHT Bump Defense";
    private final static String kLeftTwoCycleReverse = "LEFT Bump Reverse and Trench-side Jab 2 Cycle";
    private final static String kRightTwoCycleReverse = "RIGHT Bump Reverse and Trench-side Jab 2 Cycle";
    private final static String kLeftTwoCycleReverseAndJab = "LEFT Bump Reverse and Center Jab 2 Cycle";
    private final static String kRightTwoCycleReverseAndJab = "RIGHT Bump Reverse and Center Jab 2 Cycle";

    //---MISC
    private final static String kRightHubCircle = "RIGHT Bump Hub Circle";
    private final static String kLeftSweepAndDepot = "LEFT Bump Sweep and Depot";

    //---STRESS TEST
    private final static String kRightStressTestLong = "RIGHT Long Stress Test";
    private final static String kRightStressTestOverlap = "RIGHT Overlap Stress Test";
    private final static String kRightStressTestTenTimes = "RIGHT Five Times Stress Test";

    /* NEW AUTON NAMES */
    //---2 CYCLES
    private final static String kRightTrenchTwoCycleBumpReturn = "RIGHT Trench 2 Cycle Bump Return";
    private final static String kRightTrenchTwoCycleBumpReturnFollow = "RIGHT Trench 2 Cycle Bump Return FOLLOW";
    private final static String kLeftTrenchTwoCycleBumpReturn = "LEFT Trench 2 Cycle Bump Return";
    private final static String kLeftTrenchTwoCycleBumpReturnFollow = "LEFT Trench 2 Cycle Bump Return FOLLOW";
    private final static String kRightTrechTwoCycleTrenchReturn = "RIGHT Trench 2 Cycle Trench Return";
    private final static String kLeftTrenchTwoCycleTrenchReturn = "LEFT Trench 2 Cycle Trench Return";

    //---2 CYCLES PLUS OUTPOST
    private final static String kRightTrenchTwoCycleBumpReturnOutpost = "RIGHT Trench 2 Cycle Bump Return Plus Outpost";
    private final static String kRightTrenchTwoCycleTrenchReturnOutpost = "RIGHT Trench 2 Cycle Trench Return Plus Outpost";

    //---2 CYCLES PLUS DEPOT
    private final static String kLeftTrenchTwoCycleBumpReturnDepot = "LEFT Trench 2 Cycle Bump Return Plus Depot";
    private final static String kLeftTrenchTwoCycleTrenchReturnDepot = "LEFT Trench 2 Cycle Trench Return Plus Depot";

    //--MISC
    private final static String kRightTrenchSelfPass = "RIGHT Orbit";
    private final static String kLeftTrenchSelfPass = "LEFT Orbit";
    // private final static String kRightDelayTest = "DELAY TEST - NOT FOR ACTUAL USE";

    public static void initialize(WaltAdaptableAutonFactory adaptableAutonFactory) {
        m_adaptableAutonFactory = adaptableAutonFactory;
        m_chooser = new AutoChooser();
        s_multiAutons.clear();
    
        /* OLD AUTON OPTIONS */
        //---MAIN AUTONS
        addMultiAuton(kLeftShootAndSweep,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kSweepShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoSweep, AutonK.kSweepShootingTimeout, true ,0));

        addMultiAuton(kLeftShootAndPass,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoPassing, AutonK.kShootingTimeout, true, 0));

        addMultiAuton(kLeftTwoCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kShootingTimeout, false, 0));

        addMultiAuton(kRightShootAndSweep,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kSweepShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kSweepShootingTimeout, false, 0));

        addMultiAuton(kRightTwoCycle,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kShootingTimeout, false, 0));

        //---TRENCH ASSIST AUTONS
        addMultiAuton(kLeftTrenchASSISTTwoCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneTrench, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kShootingTimeout, false, 0));

        addMultiAuton(kRightTrenchASSISTTwoCycle,
            new AdaptableAutonInfo(AutonK.kRightOneTrench, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kShootingTimeout, false, 0));

        addMultiAuton(kRightShootAndPass,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoPassing, AutonK.kShootingTimeout, true, 0));

        //---DEFENSE AUTONS
        addMultiAuton(kLeftDefenseOneCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneDefense, AutonK.kShootingTimeout, false, 0));

        addMultiAuton(kRightDefenseOneCycle,
            new AdaptableAutonInfo(AutonK.kRightOneDefense, AutonK.kShootingTimeout, false, 0));

        //---DEPOT AUTONS
        addMultiAuton(kLeftShootAndDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepot, AutonK.kShootingTimeout, false, 0));

        addMultiAuton(kRightShootAndDepot,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoDepot, AutonK.kShootingTimeout, false, 0));

        //---MISC
        addMultiAuton(kRightHubCircle,
            new AdaptableAutonInfo(AutonK.kRightOneCircle, AutonK.kShootingTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kShootingTimeout, true, 0));

        addMultiAuton(kLeftSweepAndDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneSweepAndDepot, AutonK.kShootingTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kShootingTimeout, true, 0));

        addMultiAuton(kLeftTwoCycleReverse,
            new AdaptableAutonInfo(AutonK.kLeftOneReverse, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoReverse, AutonK.kShootingTimeout, false, 0));

        addMultiAuton(kRightTwoCycleReverse,
            new AdaptableAutonInfo(AutonK.kRightOneReverse, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoReverse, AutonK.kShootingTimeout, false, 0));

        addMultiAuton(kLeftTwoCycleReverseAndJab,
            new AdaptableAutonInfo(AutonK.kLeftOneReverse, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kShootingTimeout, false, 0));

        addMultiAuton(kRightTwoCycleReverseAndJab,
            new AdaptableAutonInfo(AutonK.kRightOneReverse, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kShootingTimeout, false, 0));

        /* NEW AUTON OPTIONS */
        //---2 CYCLES
        addMultiAuton(kRightTrenchTwoCycleBumpReturn,
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoGoOut, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kRightTrenchTwoCycleBumpReturnFollow,
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturnFollow, AutonK.kSOTMTimeout, true, AutonK.kFollowDelay),
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

        addMultiAuton(kLeftTrenchTwoCycleBumpReturnFollow,
            new AdaptableAutonInfo(AutonK.kLeftOneBumpReturnFollow, AutonK.kSOTMTimeout, true, AutonK.kFollowDelay),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kRightTrechTwoCycleTrenchReturn,
            new AdaptableAutonInfo(AutonK.kRightOneTrenchReturn, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoTrenchReturn, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoGoOut, AutonK.kSOTMTimeout, false, 0));

        addMultiAuton(kLeftTrenchTwoCycleTrenchReturn,
            new AdaptableAutonInfo(AutonK.kLeftOneTrenchReturn, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchReturn, AutonK.kShootingTimeout, false, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, AutonK.kSOTMTimeout, false, 0));

        //---2 CYCLES PLUS OUTPOST
        addMultiAuton(kRightTrenchTwoCycleBumpReturnOutpost,
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToOutpost, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoOutpostToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kRightTrenchTwoCycleTrenchReturnOutpost,
            new AdaptableAutonInfo(AutonK.kRightOneTrenchReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoTrenchToOutpost, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoOutpostToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kRightTwoTrenchReturn, AutonK.kSOTMTimeout, false, 0));

        //---2 CYCLES PLUS DEPOT
        addMultiAuton(kLeftTrenchTwoCycleBumpReturnDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToDepot, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepotToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, AutonK.kSOTMTimeout, true, 0));

        addMultiAuton(kLeftTrenchTwoCycleTrenchReturnDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneTrenchReturn, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchToDepot, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepotToTrench, AutonK.kSOTMTimeout, true, 0),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchReturn, AutonK.kSOTMTimeout, false, 0));

        //---MISC
        addAuton(kRightTrenchSelfPass, new AdaptableAutonInfo(AutonK.kRightOneSelfPass, AutonK.kSOTMTimeout, true, 0));
        addAuton(kLeftTrenchSelfPass, new AdaptableAutonInfo(AutonK.kLeftOneSelfPass, AutonK.kSOTMTimeout, true, 0));

        // addMultiAuton(kRightDelayTest, 
        //     new AdaptableAutonInfo(AutonK.kRightBumpPreload, AutonK.kShootingTimeout, false, 5),
        //     new AdaptableAutonInfo(AutonK.kRightOneTrench, AutonK.kShootingTimeout, false, 5));

        // addAuton(kRightStressTestLong, new AdaptableAutonInfo(AutonK.kRightStressTestLong, 100, true, 0));
        // addAuton(kRightStressTestOverlap, new AdaptableAutonInfo(AutonK.kRightStressTestOverlap, 100, true, 0));

        // addMultiAuton(kRightStressTestTenTimes,
        //     new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, 0),
        //     new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, 0));
        
        //Load AutonChooser
        SmartDashboard.putData("AutoChooser", m_chooser);
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
        names.add(kPreheatTrajectory);
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
                DriverStation.reportWarning(
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
