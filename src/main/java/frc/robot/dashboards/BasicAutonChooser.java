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
    private static SendableChooser<Command> m_chooser = new SendableChooser<Command>();
    private static WaltPointToPointAutonFactory m_autonFactory;

    public static void initialize(WaltPointToPointAutonFactory autonFactory) {
        m_autonFactory = autonFactory;
        m_chooser.setDefaultOption("Check Origin", m_autonFactory.createAuton(
            new PointToPointAutonInfo(
                new PointToPointAutonDriveInfo (
                    new Pose2d(1, 1, Rotation2d.kZero),
                    0.5,
                    1,
                    1,
                    true
                ), 5, false, false
            )
        ));
        m_chooser.addOption("more", Commands.none());
        SmartDashboard.putData(m_chooser);
    }

    public static Command getAuton() {
        return m_chooser.getSelected();
    }
}
