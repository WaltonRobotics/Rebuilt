package frc.robot.dashboards;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autons.WaltSimpleAutonFactory;

public class AutonChooser {
    public static AutoChooser m_chooser;
    public static WaltSimpleAutonFactory m_simpleAutonFactory;

    public static void initialize(WaltSimpleAutonFactory simpleAutonFactory) {
        m_simpleAutonFactory = simpleAutonFactory;
        m_chooser = new AutoChooser();

        m_chooser.addRoutine("One Right Neutral Pickup",
            () -> simpleAutonFactory.firstSweep_NoPreload(false));
        m_chooser.addRoutine("One Left Neutral Pickup",
            () -> simpleAutonFactory.firstSweep_NoPreload(true));
        m_chooser.addRoutine("Fast One Right Neutral Pickup",
            () -> simpleAutonFactory.fastOneSweep(false));
        m_chooser.addRoutine("Zigzag Two Right Neutral Pickup",
            () -> simpleAutonFactory.fastRightTwoSweep_ZigZag());
        m_chooser.addRoutine("Reshoot Two Right Neutral Pickup",
            () -> simpleAutonFactory.fastTwoSweep_Reshoot(false));
        m_chooser.addRoutine("Reshoot Two Left Neutral Pickup",
            () -> simpleAutonFactory.fastTwoSweep_Reshoot(true));
        m_chooser.addRoutine("Preload Auton",
            () -> simpleAutonFactory.preloadAuton());

        SmartDashboard.putData("AutoChooser", m_chooser);
    }

    public static Command getPreheater() {
        if (m_simpleAutonFactory == null) {
            return Commands.print("Tried to preheat before factory init!");
        }
        return m_simpleAutonFactory.preheater().cmd().ignoringDisable(true);
    }
}
