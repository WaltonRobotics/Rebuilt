package frc.robot.dashboards;

import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.units.measure.LinearVelocity;

import frc.robot.autons.WaltPointToPointAutonFactory;
import frc.robot.autons.WaltPointToPointAutonFactory.PointToPointPathDriveInfo;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.autons.WaltPointToPointAutonFactory.PointToPointPath;

public class BasicAutonChooser {
    private static SendableChooser<Command> m_chooser = new SendableChooser<Command>();
    private static WaltPointToPointAutonFactory m_autonFactory;

    private static final double kMaxTranslationSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    
    //---POSES---//
    private static final Pose2d pose_oneOneTranslation = new Pose2d(1, 1, Rotation2d.kZero);

    private static final Pose2d pose_chainTestOne = new Pose2d(8.27, 2.33, Rotation2d.fromDegrees(0.0));
    private static final Pose2d pose_chainTestTwo = new Pose2d(7.33, 4.80, Rotation2d.fromDegrees(0.0));

    private static final Pose2d pose_intakeTest = new Pose2d(9.63, 0.88, Rotation2d.fromDegrees(-90.0));
    private static final Pose2d pose_shootingTest = new Pose2d(8.29, 6.92, Rotation2d.fromDegrees(270.0));
    private static final Pose2d pose_passingTest = new Pose2d(8.22, 1.48, Rotation2d.fromDegrees(270.0));

    //---PATHS---//
    private static final PointToPointPath path_oneOneTranslation = new PointToPointPath(
        new PointToPointPathDriveInfo (
            pose_oneOneTranslation,
            0.5,
            1,
            1,
            true
        ), 5, false, false
    );

    private static final PointToPointPath path_chainTestOne = new PointToPointPath(
        new PointToPointPathDriveInfo(
            pose_chainTestOne,
            0.1,
            kMaxTranslationSpeed,
            1,
            true
        ), 10, false, false
    );
    private static final PointToPointPath path_chainTestTwo = new PointToPointPath(
        new PointToPointPathDriveInfo(
            pose_chainTestTwo,
            0.1,
            kMaxTranslationSpeed,
            1,
            false
        ), 10, false, false
    );

    private static final PointToPointPath path_intakeTest = new PointToPointPath(
        new PointToPointPathDriveInfo(
            pose_intakeTest,
            0.1,
            1,
            1,
            true
        ), 10, true, false
    );
    private static final PointToPointPath path_shootingTest = new PointToPointPath(
        new PointToPointPathDriveInfo(
            pose_shootingTest,
            0.1,
            1,
            1,
            true
        ), 10, false, true
    );
    private static final PointToPointPath path_passingTest = new PointToPointPath(
        new PointToPointPathDriveInfo(
            pose_passingTest,
            0.1,
            1,
            1,
            false
        ), 10, true, true
    );

    //---AUTONS---//
    private static final PointToPointAuton auto_checkOrigin = new PointToPointAuton(
        "Check Origin",
        m_autonFactory.createAuton(path_oneOneTranslation)
    );

    private static final PointToPointAuton auto_checkChaining = new PointToPointAuton(
        "Check Chaining",
        m_autonFactory.createAuton(
            path_chainTestOne,
            path_chainTestTwo
        )
    );

    private static final PointToPointAuton auto_subsystemsTest = new PointToPointAuton(
        "Subsystem Test",
        m_autonFactory.createAuton(
            path_intakeTest,
            path_shootingTest,
            path_passingTest
        )
    );

    private static final PointToPointAuton auto_doNothing = new PointToPointAuton(
        "Do Nothing",
        Commands.none()
    );

    //---FUNCTIONALITIES---//
    public static void initialize(WaltPointToPointAutonFactory autonFactory) {
        m_autonFactory = autonFactory;

        setDefaultAuton(auto_checkOrigin);
        addAuton(auto_checkChaining);
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
