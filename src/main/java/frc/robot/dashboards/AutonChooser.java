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

    //---STRESS TEST
    private final static String kRightStressTestLong = "RIGHT Long Stress Test";
    private final static String kRightStressTestOverlap = "RIGHT Overlap Stress Test";
    private final static String kRightStressTestFiveTimes = "RIGHT Five Times Stress Test";


    public static void initialize(WaltAdaptableAutonFactory adaptableAutonFactory) {
        m_adaptableAutonFactory = adaptableAutonFactory;
        m_chooser = new AutoChooser();
    
        /* AUTON OPTIONS */
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

        m_chooser.addRoutine(kRightStressTestFiveTimes,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightStressTestFiveTimes,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightBeanTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightBeanTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightBeanTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightBeanTrench, 100, true),
                                          new AdaptableAutonInfo(AutonK.kRightBeanTrench, 100, true)}));

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
