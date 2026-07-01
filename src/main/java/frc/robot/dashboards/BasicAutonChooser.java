package frc.robot.dashboards;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

public class BasicAutonChooser {
    private SendableChooser<Command> m_chooser = new SendableChooser<Command>();

    public BasicAutonChooser() {
        initAutonChooser();
    }

    public void initAutonChooser() {
        m_chooser.setDefaultOption("test", Commands.none());
        m_chooser.addOption("more", Commands.none());
        SmartDashboard.putData(m_chooser);
    }

    public Command getAuton() {
        return m_chooser.getSelected();
    }
}
