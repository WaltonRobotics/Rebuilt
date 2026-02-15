package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.DeployPosition;
import frc.robot.subsystems.Intake.RollersVelocity;
import frc.util.WaltLogger;
import frc.util.WaltLogger.IntLogger;
import frc.util.WaltLogger.StringLogger;

import static frc.robot.Constants.RobotK.*;
import static frc.robot.Constants.ShooterK.kFlywheelLowRPS;
import static frc.robot.Constants.ShooterK.kFlywheelMaxRPS;
import static frc.robot.Constants.ShooterK.kFlywheelZeroRPS;

public class Superstructure {

    /* declare subsystems */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    /* constructor */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* button bind sequences */
    public Command prepIntake() {
        return Commands.sequence(
            m_indexer.stopSpinner(),
            m_intake.setDeployPos(DeployPosition.SAFE),
            m_intake.setRollersSpeed(RollersVelocity.STOP)
        );
    }

    public Command activateIntake() {
        return Commands.sequence(
            m_intake.setRollersSpeed(RollersVelocity.MAX),
            m_indexer.startSpinner(),
            m_intake.setDeployPos(DeployPosition.DEPLOYED)
        );
    }

    public Command retractIntake() {
        return Commands.sequence(
            m_indexer.stopSpinner(),
            m_intake.setDeployPos(DeployPosition.RETRACTED),
            m_intake.setRollersSpeed(RollersVelocity.STOP)
        );
    }

    public Command activateOuttake(AngularVelocity rps) {
        return Commands.sequence(
            m_indexer.startSpinner(),
            m_indexer.startExhaust(),
            m_shooter.setFlywheelVelocityCmd(rps)
        );
    }

    public Command deactivateOuttake() {
        return Commands.sequence(
            m_indexer.stopSpinner(),
            m_indexer.stopExhaust(),
            m_shooter.setFlywheelVelocityCmd(kFlywheelZeroRPS)
        );
    }

    public Command normalOuttake() {
        return activateOuttake(kFlywheelMaxRPS);
    }

    public Command emergencyOuttake() {
        return activateOuttake(kFlywheelLowRPS);
    }

    public Command startPassing() {
        return Commands.sequence(
            activateIntake(),
            activateOuttake(kFlywheelMaxRPS)
        );
    }

    public Command stopPassing() {
        return Commands.sequence(
            prepIntake(),
            deactivateOuttake()
        );
    }

    public Command prepExhaust() {
        return Commands.sequence(
            m_indexer.startSpinner(),
            m_indexer.startExhaust(),
            Commands.waitSeconds(0.3),
            m_indexer.stopSpinner(),
            m_indexer.stopExhaust()
        );
    }
}