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

    public Command createAuton(PointToPointAutonInfo... infos) {
        SequentialCommandGroup returnCommand = new SequentialCommandGroup();
        for (PointToPointAutonInfo info : infos) {
            ParallelCommandGroup subsystemsCommand = new ParallelCommandGroup();
            if (info.intaking) {
                subsystemsCommand.addCommands(m_superstructure.intake(() -> info.shooting, () -> false));
            }
            if (info.shooting) {
                subsystemsCommand.addCommands(m_superstructure.activateOuttakeShotCalc());
                if (!info.intaking) {
                    subsystemsCommand.addCommands(m_superstructure.intakeShimmy(() -> info.shooting));
                }
            }
            
            returnCommand.addCommands(
                Commands.race(
                    m_swerve.driveToPoint(
                        info.driveInfo.target, 
                        info.driveInfo.tolerance, 
                        info.driveInfo.maxVel, 
                        info.driveInfo.maxRVel, 
                        info.driveInfo.isContinuous
                    ),
                    Commands.waitSeconds(info.timeout)
                ).alongWith(subsystemsCommand)
            );
        }
        return returnCommand;
    }

    public final record PointToPointAutonInfo(
        PointToPointAutonDriveInfo driveInfo,
        double timeout,
        boolean intaking,
        boolean shooting
    ) {}

    public final record PointToPointAutonDriveInfo(
        Pose2d target, 
        double tolerance, 
        double maxVel, 
        double maxRVel, 
        // Translation2d robotCenterComp, 
        boolean isContinuous
    ) {}
}
