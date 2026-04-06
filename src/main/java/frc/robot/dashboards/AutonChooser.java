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

    //---AUTON NAMES
    private final static String kRightShootAndSweep = "RIGHT 1.5 Cycle";
    private final static String kLeftShootAndSweep = "LEFT 1.5 Cycle";
    private final static String kLeftTwoCycle = "LEFT 2 Cycle";
    private final static String kRightTwoCycle = "RIGHT 2 Cycle";
    private final static String kRightHubCircle = "RIGHT Hub Circle";
    private final static String kRightTrenchTwoCycle = "RIGHT Trench 2 Cycle";
    private final static String kLeftTrenchTwoCycle = "LEFT Trench 2 Cycle";

    public static void initialize(WaltAdaptableAutonFactory adaptableAutonFactory) {
        m_adaptableAutonFactory = adaptableAutonFactory;
        m_chooser = new AutoChooser();
    
        //---AUTON OPTIONS
        m_chooser.addRoutine(kRightShootAndSweep,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightShootAndSweep,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kSweepShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kSweepShootingTimeout, false)}));
        m_chooser.addRoutine(kLeftShootAndSweep,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftShootAndSweep,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kSweepShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoSweep, AutonK.kSweepShootingTimeout, true)}));

        m_chooser.addRoutine(kLeftTwoCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTwoCycle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneJab, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kRightHubCircle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightHubCircle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneCircle, AutonK.kReshootShootingTimeout, true),
                                          new AdaptableAutonInfo(AutonK.kRightTwoSweep, AutonK.kReshootShootingTimeout, true)}));

        m_chooser.addRoutine(kRightTwoCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTwoCycle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneJab, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kLeftTrenchTwoCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kLeftTrenchTwoCycle, 
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kLeftOneTrench, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kLeftTwoJab, AutonK.kReshootShootingTimeout, false)}));
        
        m_chooser.addRoutine(kRightTrenchTwoCycle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightTrenchTwoCycle, 
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kRightOneTrench, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kRightTwoJab, AutonK.kReshootShootingTimeout, false)}));

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
