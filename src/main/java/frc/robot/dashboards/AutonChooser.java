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
                new AdaptableAutonInfo(AutonK.kSweepLeftTwo, AutonK.kShootingTimeout, false)}));

        SmartDashboard.putData("AutoChooser", m_chooser);
    }

    public static Command getPreheater() {
        if (m_simpleAutonFactory == null) {
            return Commands.print("Tried to preheat before factory init!");
        }
        return m_simpleAutonFactory.preheater().cmd().ignoringDisable(true);
    }
}
