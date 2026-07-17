package frc.robot.dashboards;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import frc.robot.autons.WaltPointToPointAutonFactory;
import frc.robot.autons.WaltPointToPointAutonFactory.PointToPointPathDriveInfo;
import frc.robot.autons.WaltPointToPointAutonFactory.PointToPointPath;

public class BasicAutonChooser {
    private static SendableChooser<Command> m_chooser = new SendableChooser<Command>();
    private static WaltPointToPointAutonFactory m_autonFactory;
    
    // POSES
    private static final Pose2d pose_oneOneTranslation = new Pose2d(1, 1, Rotation2d.kZero);

    // PATHS
    private static final PointToPointPath path_oneOneTranslation = new PointToPointPath(
        new PointToPointPathDriveInfo (
            pose_oneOneTranslation,
            0.5,
            1,
            1,
            true
        ), 5, false, false
    );

    // AUTONS
    private static final PointToPointAuton auto_checkOrigin = new PointToPointAuton(
        "Check Origin",
        m_autonFactory.createAuton(path_oneOneTranslation)
    );

    private static final PointToPointAuton auto_doNothing = new PointToPointAuton(
        "Do Nothing",
        Commands.none()
    );

    // FUNCTIONALITIES
    public static void initialize(WaltPointToPointAutonFactory autonFactory) {
        m_autonFactory = autonFactory;

        setDefaultAuton(auto_checkOrigin);
        addAuton(auto_doNothing);

        SmartDashboard.putData(m_chooser);
    }

    private static void setDefaultAuton(PointToPointAuton auton) {
        m_chooser.setDefaultOption(auton.name, auton.auton);
    }

    private static void addAuton(PointToPointAuton auton) {
        m_chooser.addOption(auton.name, auton.auton);
    }

    public static Command getAuton() {
        return m_chooser.getSelected();
    }

    public record PointToPointAuton(
        String name,
        Command auton
    ) {}
}
