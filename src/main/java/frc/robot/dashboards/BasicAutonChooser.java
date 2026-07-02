package frc.robot.dashboards;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import frc.robot.autons.WaltPointToPointAutonFactory;
import frc.robot.autons.WaltPointToPointAutonFactory.PointToPointAutonDriveInfo;
import frc.robot.autons.WaltPointToPointAutonFactory.PointToPointAutonInfo;

public class BasicAutonChooser {
    private SendableChooser<Command> m_chooser = new SendableChooser<Command>();
    private WaltPointToPointAutonFactory m_autonFactory = new WaltPointToPointAutonFactory(null, null, null, null);

    public BasicAutonChooser() {
        initAutonChooser();
    }

    public void initAutonChooser() {
        m_chooser.setDefaultOption("Check Origin", m_autonFactory.createAuton(
            new PointToPointAutonInfo(
                new PointToPointAutonDriveInfo (
                    new Pose2d(1, 1, Rotation2d.kZero),
                    0.5,
                    1,
                    1,
                    false
                ), 5
            )
        ));
        m_chooser.addOption("more", Commands.none());
        SmartDashboard.putData(m_chooser);
    }

    public Command getAuton() {
        return m_chooser.getSelected();
    }
}
