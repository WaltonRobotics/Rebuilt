package frc.robot.dashboards;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.autons.WaltAdaptableAutonFactory;
import frc.robot.autons.WaltAdaptableAutonFactory.AdaptableAutonInfo;

public class AutonChooser {
    public static AutoChooser m_chooser;
    public static WaltAdaptableAutonFactory m_adaptableAutonFactory;

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
    
        /* OLD AUTON OPTIONS */
        //---MAIN AUTONS
        m_chooser.addRoutine(kLeftShootAndSweep,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftShootAndSweep,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kSweepShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoSweep, AutonK.kSweepShootingTimeout, true)}));
        
        m_chooser.addRoutine(kLeftShootAndPass,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftShootAndPass, 
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoPassing, AutonK.kReshootShootingTimeout, true)}));

        m_chooser.addRoutine(kLeftTwoCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTwoCycle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kRightShootAndSweep,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightShootAndSweep,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kSweepShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kSweepShootingTimeout, false)}));

        m_chooser.addRoutine(kRightTwoCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTwoCycle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false)}));

        //---TRENCH ASSIST AUTONS
        m_chooser.addRoutine(kLeftTrenchTwoCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTrenchTwoCycle, 
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneTrench, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false)}));
        
        m_chooser.addRoutine(kRightTrenchTwoCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTrenchTwoCycle, 
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneTrench, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kRightShootAndPass,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightShootAndPass, 
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoPassing, AutonK.kReshootShootingTimeout, true)}));
        
        //---DEFENSE AUTONS
        m_chooser.addRoutine(kLeftDefenseOneCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftDefenseOneCycle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneDefense, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kRightDefenseOneCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightDefenseOneCycle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneDefense, AutonK.kReshootShootingTimeout, false)}));
        
        //---DEPOT AUTONS
        m_chooser.addRoutine(kLeftShootAndDepot,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftShootAndDepot, 
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoDepot, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kRightShootAndDepot,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightShootAndDepot, 
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoDepot, AutonK.kReshootShootingTimeout, false)}));

        //---MISC
        m_chooser.addRoutine(kRightHubCircle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightHubCircle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneCircle, AutonK.kReshootShootingTimeout, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kReshootShootingTimeout, true)}));
        
        m_chooser.addRoutine(kLeftSweepAndDepot,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftSweepAndDepot,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneSweepAndDepot, AutonK.kReshootShootingTimeout, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, true)}));

        m_chooser.addRoutine(kLeftTwoCycleReverse,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTwoCycleReverse,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneReverse, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoReverse, AutonK.kReshootShootingTimeout, false)}));
        
        m_chooser.addRoutine(kRightTwoCycleReverse,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTwoCycleReverse,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneReverse, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoReverse, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kLeftTwoCycleReverseAndJab,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTwoCycleReverseAndJab,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneReverse, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false)}));
        
        m_chooser.addRoutine(kRightTwoCycleReverseAndJab,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTwoCycleReverseAndJab,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneReverse, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kRightStressTestLong,
            () -> m_adaptableAutonFactory.adaptableAuton(kRightStressTestLong, new AdaptableAutonInfo(AutonK.kRightStressTestLong, 100, true)));
        
        m_chooser.addRoutine(kRightStressTestOverlap,
            () -> m_adaptableAutonFactory.adaptableAuton(kRightStressTestOverlap, new AdaptableAutonInfo(AutonK.kRightStressTestOverlap, 100, true)));

        m_chooser.addRoutine(kRightStressTestTenTimes,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightStressTestTenTimes,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true)}));

        /* NEW AUTON OPTIONS */
        //---2 CYCLES
        m_chooser.addRoutine(kRightTwoCycleBumpReturn,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTwoCycleBumpReturn,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoGoOut, 100, true)}));

        m_chooser.addRoutine(kLeftTwoCycleBumpReturn,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTwoCycleBumpReturn,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, 100, true)}));

        m_chooser.addRoutine(kRightTwoCycleTrenchReturn,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTwoCycleTrenchReturn,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneTrenchReturn, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoTrenchReturn, 100, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoGoOut, 100, false)}));

        m_chooser.addRoutine(kLeftTwoCycleTrenchReturn,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTwoCycleTrenchReturn,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneTrenchReturn, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoTrenchReturn, 100, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoGoOut, 100, false)}));

        //---2 CYCLES PLUS OUTPOST
        m_chooser.addRoutine(kRightTwoCycleBumpReturnOutpost,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTwoCycleBumpReturnOutpost,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToOutpost, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoOutpostToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoBumpToTrench, 100, true)}));

        m_chooser.addRoutine(kRightTwoCycleTrenchReturnOutpost,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTwoCycleTrenchReturnOutpost,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneTrenchReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoTrenchToOutpost, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoOutpostToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoTrenchReturn, 100, false)}));

        //---2 CYCLES PLUS DEPOT
        m_chooser.addRoutine(kLeftTwoCycleBumpReturnDepot,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTwoCycleBumpReturnDepot,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoBumpToDepot, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoDepotToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoBumpReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoBumpToTrench, 100, true)}));

        m_chooser.addRoutine(kLeftTwoCycleTrenchReturnDepot,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTwoCycleTrenchReturnDepot,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneTrenchReturn, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoTrenchToDepot, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoDepotToTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoTrenchReturn, 100, false)}));

        //---MISC
        m_chooser.addRoutine(kRightOrbit,
            () -> m_adaptableAutonFactory.adaptableAuton(kRightOrbit,
                new AdaptableAutonInfo(AutonK.kRightOneSelfPass, 100, true)));

        m_chooser.addRoutine(kLeftOrbit,
            () -> m_adaptableAutonFactory.adaptableAuton(kLeftOrbit,
                new AdaptableAutonInfo(AutonK.kLeftOneSelfPass, 100, true)));
        
        //Load AutonChooser
        SmartDashboard.putData("AutoChooser", m_chooser);
    }

    public static Command getPreheater() {
        if (m_adaptableAutonFactory == null) {
            return Commands.print("Tried to preheat before factory init!");
        }
        
        return m_adaptableAutonFactory.preheater().cmd().ignoringDisable(true);
    }
}
