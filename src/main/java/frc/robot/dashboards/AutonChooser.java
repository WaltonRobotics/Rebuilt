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

    //---STRESS TEST
    private final static String kRightStressTestLong = "RIGHT Long Stress Test";
    private final static String kRightStressTestOverlap = "RIGHT Overlap Stress Test";
    private final static String kRightStressTestTenTimes = "RIGHT Five Times Stress Test";

    /* NEW AUTON NAMES */
    //---2 CYCLES
    private final static String kRightTwoCycleBumpReturn = "RIGHT Two Cycle Bump Return";
    private final static String kLeftTwoCycleBumpReturn = "LEFT Two Cycle Bump Return";
    private final static String kRightTwoCycleTrenchReturn = "RIGHT Two Cycle Trench Return";
    private final static String kLeftTwoCycleTrenchReturn = "LEFT Two Cycle Trench Return";

    //---2 CYCLES PLUS OUTPOST
    private final static String kRightTwoCycleBumpReturnOutpost = "RIGHT Two Cycle Bump Return Plus Outpost";
    private final static String kRightTwoCycleTrenchReturnOutpost = "RIGHT Two Cycle Trench Return Plus Outpost";

    //---2 CYCLES PLUS DEPOT
    private final static String kLeftTwoCycleBumpReturnDepot = "LEFT Two Cycle Bump Return Plus Depot";
    private final static String kLeftTwoCycleTrenchReturnDepot = "LEFT Two Cycle Trench Return Plus Depot";

    //--MISC
    private final static String kRightOrbit = "RIGHT Orbit";
    private final static String kLeftOrbit = "LEFT Orbit";

    public static void initialize(WaltAdaptableAutonFactory adaptableAutonFactory) {
        m_adaptableAutonFactory = adaptableAutonFactory;
        m_chooser = new AutoChooser();
        s_multiAutons.clear();
    
        /* OLD AUTON OPTIONS */
        //---MAIN AUTONS
        addMultiAuton(kLeftShootAndSweep,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kSweepShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoSweep, AutonK.kSweepShootingTimeout, true ,false));

        addMultiAuton(kLeftShootAndPass,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoPassing, AutonK.kReshootShootingTimeout, true, false));

        addMultiAuton(kLeftTwoCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false, false));

        addMultiAuton(kRightShootAndSweep,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kSweepShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kSweepShootingTimeout, false, false));

        addMultiAuton(kRightTwoCycle,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false, false));

        //---TRENCH ASSIST AUTONS
        addMultiAuton(kLeftTrenchTwoCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneTrench, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false, false));

        addMultiAuton(kRightTrenchTwoCycle,
            new AdaptableAutonInfo(AutonK.kRightOneTrench, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false, false));

        addMultiAuton(kRightShootAndPass,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoPassing, AutonK.kReshootShootingTimeout, true, false));

        //---DEFENSE AUTONS
        addMultiAuton(kLeftDefenseOneCycle,
            new AdaptableAutonInfo(AutonK.kLeftOneDefense, AutonK.kReshootShootingTimeout, false, false));

        addMultiAuton(kRightDefenseOneCycle,
            new AdaptableAutonInfo(AutonK.kRightOneDefense, AutonK.kReshootShootingTimeout, false, false));

        //---DEPOT AUTONS
        addMultiAuton(kLeftShootAndDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepot, AutonK.kReshootShootingTimeout, false, false));

        addMultiAuton(kRightShootAndDepot,
            new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoDepot, AutonK.kReshootShootingTimeout, false, false));

        //---MISC
        addMultiAuton(kRightHubCircle,
            new AdaptableAutonInfo(AutonK.kRightOneCircle, AutonK.kReshootShootingTimeout, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kReshootShootingTimeout, true, false));

        addMultiAuton(kLeftSweepAndDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneSweepAndDepot, AutonK.kReshootShootingTimeout, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, true, false));

        addMultiAuton(kLeftTwoCycleReverse,
            new AdaptableAutonInfo(AutonK.kLeftOneReverse, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoReverse, AutonK.kReshootShootingTimeout, false, false));

        addMultiAuton(kRightTwoCycleReverse,
            new AdaptableAutonInfo(AutonK.kRightOneReverse, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoReverse, AutonK.kReshootShootingTimeout, false, false));

        addMultiAuton(kLeftTwoCycleReverseAndJab,
            new AdaptableAutonInfo(AutonK.kLeftOneReverse, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false, false));

        addMultiAuton(kRightTwoCycleReverseAndJab,
            new AdaptableAutonInfo(AutonK.kRightOneReverse, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false, false));

        addAuton(kRightStressTestLong, new AdaptableAutonInfo(AutonK.kRightStressTestLong, 100, true, false));
        
        addAuton(kRightStressTestOverlap, new AdaptableAutonInfo(AutonK.kRightStressTestOverlap, 100, true, false));

        addMultiAuton(kRightStressTestTenTimes,
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, false));

        /* NEW AUTON OPTIONS */
        //---2 CYCLES
        addMultiAuton(kRightTwoCycleBumpReturn,
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoGoOut, 100, true, false));

        addMultiAuton(kLeftTwoCycleBumpReturn,
            new AdaptableAutonInfo(AutonK.kLeftOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, 100, true, false));

        addMultiAuton(kRightTwoCycleTrenchReturn,
            new AdaptableAutonInfo(AutonK.kRightOneTrenchReturn, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoTrenchReturn, 100, false, false),
            new AdaptableAutonInfo(AutonK.kRightTwoGoOut, 100, false, false));

        addMultiAuton(kLeftTwoCycleTrenchReturn,
            new AdaptableAutonInfo(AutonK.kLeftOneTrenchReturn, AutonK.kReshootShootingTimeout, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchReturn, 100, false, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, 100, false, false));

        //---2 CYCLES PLUS OUTPOST
        addMultiAuton(kRightTwoCycleBumpReturnOutpost,
            new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToOutpost, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoOutpostToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true, false));

        addMultiAuton(kRightTwoCycleTrenchReturnOutpost,
            new AdaptableAutonInfo(AutonK.kRightOneTrenchReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoTrenchToOutpost, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoOutpostToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kRightTwoTrenchReturn, 100, false, false));

        //---2 CYCLES PLUS DEPOT
        addMultiAuton(kLeftTwoCycleBumpReturnDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToDepot, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepotToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, 100, true, false));

        addMultiAuton(kLeftTwoCycleTrenchReturnDepot,
            new AdaptableAutonInfo(AutonK.kLeftOneTrenchReturn, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchToDepot, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoDepotToTrench, 100, true, false),
            new AdaptableAutonInfo(AutonK.kLeftTwoTrenchReturn, 100, false, false));

        //---MISC
        addAuton(kRightOrbit, new AdaptableAutonInfo(AutonK.kRightOneSelfPass, 100, true, false));

        addMultiAuton(kLeftOrbit, new AdaptableAutonInfo(AutonK.kLeftOneSelfPass, 100, true, false));
        
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
        for (AutonEntry e : s_autons) {
            m_adaptableAutonFactory.adaptableAuton(e.name(), e.infos());
        }
        for (MultiAutonEntry e : s_multiAutons) {
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
