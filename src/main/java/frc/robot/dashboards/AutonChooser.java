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
    private final static String kTwoRightSweep = "Two RIGHT Sweep";
    private final static String kTwoLeftSweep = "Two LEFT Sweep";
    private final static String kTwoLeftCycleReshoot = "Two LEFT Cycle Reshoot";
    private final static String kTwoRightCycleReshoot = "TWO RIGHT Cycle Reshoot";
    private final static String kSOTMFullHubCircle = "SOTM Full Hub Circle";
    private final static String kRightDoubleJab = "RIGHT double jab";

    public static void initialize(WaltAdaptableAutonFactory adaptableAutonFactory) {
        m_adaptableAutonFactory = adaptableAutonFactory;
        m_chooser = new AutoChooser();
    
        //---AUTON OPTIONS
        m_chooser.addRoutine(kTwoRightSweep,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kTwoRightSweep,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastRightSweep, AutonK.kSweepShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kSweepLeftTwo, AutonK.kSweepShootingTimeout, false)}));
        m_chooser.addRoutine(kTwoLeftSweep,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kTwoLeftSweep,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastLeftSweep, AutonK.kSweepShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kSweepLeftTwo, AutonK.kSweepShootingTimeout, true)}));

        m_chooser.addRoutine(kTwoLeftCycleReshoot,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kTwoLeftCycleReshoot,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastLeftSweep, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kReshootLeftTwo, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kTwoRightCycleReshoot,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kTwoRightCycleReshoot,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastRightSweep, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kReshootRightTwo, AutonK.kReshootShootingTimeout, false)}));

        m_chooser.addRoutine(kSOTMFullHubCircle,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kSOTMFullHubCircle,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kHubCircle, AutonK.kReshootShootingTimeout, true),
                                          new AdaptableAutonInfo(AutonK.kSweepRightTwo, AutonK.kReshootShootingTimeout, true)}));

        m_chooser.addRoutine(kRightDoubleJab,
            () -> m_adaptableAutonFactory.multiAdaptableAuton(kRightDoubleJab,
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastRightSweep, AutonK.kReshootShootingTimeout, false),
                                          new AdaptableAutonInfo(AutonK.kFastRightTwo, AutonK.kReshootShootingTimeout, false)}));

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
