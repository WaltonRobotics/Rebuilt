package frc.robot.dashboards;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.autons.WaltAdaptableAutonFactory;
import frc.robot.autons.WaltSimpleAutonFactory;
import frc.robot.autons.WaltAdaptableAutonFactory.AdaptableAutonInfo;

public class AutonChooser {
    public static AutoChooser m_chooser;
    public static WaltSimpleAutonFactory m_simpleAutonFactory;
    public static WaltAdaptableAutonFactory m_adaptableAutonFactory;
    public static AdaptableAutonInfo m_traj1;
    public static AdaptableAutonInfo m_traj2;

    public static void initialize(WaltSimpleAutonFactory simpleAutonFactory, WaltAdaptableAutonFactory adaptableAutonFactory) {
        m_simpleAutonFactory = simpleAutonFactory;
        m_adaptableAutonFactory = adaptableAutonFactory;
        m_chooser = new AutoChooser();
    
        m_chooser.addRoutine("Two Right Sweep",
            () -> m_adaptableAutonFactory.multiAdaptableAuton("Two Right Sweep",
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastRightSweep, AutonK.kShootingTimeout, false),
                new AdaptableAutonInfo(AutonK.kSweepLeftTwo, AutonK.kShootingTimeout, false)}));
        m_chooser.addRoutine("Two Left Sweep",
            () -> m_adaptableAutonFactory.multiAdaptableAuton("Two Left Sweep",
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastLeftSweep, AutonK.kShootingTimeout, false),
                new AdaptableAutonInfo(AutonK.kSweepLeftTwo, AutonK.kShootingTimeout, true)}));

        m_chooser.addRoutine("SOTM Two Cycle Reshoot",
            () -> m_adaptableAutonFactory.multiAdaptableAuton("SOTM Two Cycle Reshoot",
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastLeftSweep, AutonK.kShootingTimeout, false),
                                        //   new AdaptableAutonInfo(AutonK.kDepotLeftTwo, AutonK.kShootingTimeout, true),
                                          new AdaptableAutonInfo(AutonK.kReshootLeftTwo, AutonK.kShootingTimeout, false)}));

        m_chooser.addRoutine("SOTM Two Cycle Sweep",
            () -> m_adaptableAutonFactory.multiAdaptableAuton("SOTM Two Cycle Sweep",
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kFastLeftSweep, AutonK.kShootingTimeout, false),
                                        //   new AdaptableAutonInfo(AutonK.kDepotLeftTwo, AutonK.kShootingTimeout, true),
                                          new AdaptableAutonInfo(AutonK.kSweepLeftTwo, AutonK.kShootingTimeout, true)}));

        m_chooser.addRoutine("SOTM Full Hub Circle",
            () -> m_adaptableAutonFactory.multiAdaptableAuton("SOTM Full Hub Circle",
                new AdaptableAutonInfo[] {new AdaptableAutonInfo(AutonK.kHubCircle, AutonK.kShootingTimeout, true),
                                          new AdaptableAutonInfo(AutonK.kSweepRightTwo, 0, true)}));

        SmartDashboard.putData("AutoChooser", m_chooser);
    }

    public static Command getPreheater() {
        if (m_simpleAutonFactory == null) {
            return Commands.print("Tried to preheat before factory init!");
        }
        return m_simpleAutonFactory.preheater().cmd().ignoringDisable(true);
    }
}
