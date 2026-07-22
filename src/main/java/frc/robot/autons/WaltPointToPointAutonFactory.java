package frc.robot.autons;

import org.opencv.core.Point;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;

public class WaltPointToPointAutonFactory {
    private Superstructure m_superstructure;
    private Intake m_intake;
    private Shooter m_shooter;
    private Swerve m_swerve;

    public WaltPointToPointAutonFactory(Superstructure superstructure, Intake intake, Shooter shooter, Swerve swerve) {
        m_superstructure = superstructure;
        m_intake = intake;
        m_shooter = shooter;
        m_swerve = swerve;
    }

    public Command createAuton(PointToPointPath... paths) {
        SequentialCommandGroup returnCommand = new SequentialCommandGroup();
        for (PointToPointPath path : paths) {
            ParallelCommandGroup subsystemsCommand = new ParallelCommandGroup();
            if (path.intaking) {
                subsystemsCommand.addCommands(m_superstructure.intake(() -> path.shooting, () -> false));
            }
            if (path.shooting) {
                subsystemsCommand.addCommands(m_superstructure.activateOuttakeShotCalc());
                if (!path.intaking) {
                    subsystemsCommand.addCommands(m_superstructure.intakeShimmy(() -> path.shooting));
                }
            }
            
            returnCommand.addCommands(
                Commands.race(
                    m_swerve.driveToPoint(
                        path.driveInfo.target, 
                        path.driveInfo.tolerance, 
                        path.driveInfo.maxVel, 
                        path.driveInfo.maxRVel, 
                        path.driveInfo.isContinuous
                    ),
                    Commands.waitSeconds(path.timeout),
                    subsystemsCommand
                )
            );
        }
        return returnCommand;
    }

    public final record PointToPointPath(
        PointToPointPathDriveInfo driveInfo,
        double timeout,
        boolean intaking,
        boolean shooting
    ) {}

    public final record PointToPointPathDriveInfo(
        Pose2d target, 
        double tolerance, 
        double maxVel, 
        double maxRVel, 
        // Translation2d robotCenterComp, 
        boolean isContinuous
    ) {}
}
