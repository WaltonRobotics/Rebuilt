package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.DeployPosition;
import frc.robot.subsystems.Intake.RollersVelocity;
import frc.util.WaltLogger;
import frc.util.WaltLogger.StringArrayLogger;

import static edu.wpi.first.units.Units.Degree;
import static frc.robot.Constants.RobotK.*;
import static frc.robot.Constants.ShooterK.kFlywheelLowRPS;
import static frc.robot.Constants.ShooterK.kFlywheelMaxRPS;
import static frc.robot.Constants.ShooterK.kFlywheelZeroRPS;

import java.util.ArrayList;

public class Superstructure {

    /* declare subsystems */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    /* loggers */
    private ArrayList<String> activeCommands = new ArrayList<>();
    private final StringArrayLogger log_activeCommands = WaltLogger.logStringArray(kLogTab, "Active Commands");

    private ArrayList<String> activeOverrideCommands = new ArrayList<>();
    private final StringArrayLogger log_activeOverrideCommands = WaltLogger.logStringArray(kLogTab, "Active Override Commands");
    
    /* constructor */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* button bind sequences */

    // Regular Commands
    public Command prepIntake() {
        logActiveCommands("prepIntake", "activateIntake", "retractIntake");
        return Commands.sequence(
            m_indexer.stopSpinner(),
            m_intake.setDeployPos(DeployPosition.SAFE),
            m_intake.setRollersSpeed(RollersVelocity.STOP)
        );
    }

    public Command activateIntake() {
        logActiveCommands("activateIntake", "prepIntake", "retractIntake");
        return Commands.sequence(
            m_intake.setRollersSpeed(RollersVelocity.MAX),
            m_indexer.startSpinner(),
            m_intake.setDeployPos(DeployPosition.DEPLOYED)
        );
    }

    public Command retractIntake() {
        logActiveCommands("retractIntake", "activateIntake", "prepIntake");
        return Commands.sequence(
            m_indexer.stopSpinner(),
            m_intake.setDeployPos(DeployPosition.RETRACTED),
            m_intake.setRollersSpeed(RollersVelocity.STOP)
        );
    }

    private Command activateOuttake(AngularVelocity rps) {
        return Commands.sequence(
            m_indexer.startSpinner(),
            m_indexer.startExhaust(),
            m_shooter.setFlywheelVelocityCmd(rps)
        );
    }

    public Command normalOuttake() {
        logActiveCommands("normalOuttake", "deactivateOuttake", "emergencyOuttake");
        return activateOuttake(kFlywheelMaxRPS);
    }

    public Command emergencyOuttake() {
        logActiveCommands("emergencyOuttake", "normalOuttake", "deactivateOuttake");
        return activateOuttake(kFlywheelLowRPS);
    }

    public Command deactivateOuttake() {
            logActiveCommands("deactivateOuttake", "normalOuttake", "emergencyOuttake");
            return Commands.sequence(
                m_indexer.stopSpinner(),
                m_indexer.stopExhaust(),
                m_shooter.setFlywheelVelocityCmd(kFlywheelZeroRPS)
            );
        }
    
    public Command startPassing() {
        logActiveCommands("startPassing", "stopPassing");
        return Commands.sequence(
            activateIntake(),
            activateOuttake(kFlywheelMaxRPS)
        );
    }

    public Command stopPassing() {
        logActiveCommands("stopPassing", "startPassing");
        return Commands.sequence(
            prepIntake(),
            deactivateOuttake()
        );
    }

    /**
     * Adds and removes specified Command names from the ActiveCommands ArrayList, then logs the ArrayList.
     * @param toAdd Command name to add.
     * @param toRemove Command names to remove.
     */
    private void logActiveCommands(String toAdd, String... toRemove) {
        if (!activeCommands.contains(toAdd)) {
            activeCommands.add(toAdd);
        }
        for (String r : toRemove) {
            activeCommands.remove(r);
        }
        log_activeCommands.accept(activeCommands.toArray(new String[activeCommands.size()]));
    }

    // Override commands
    public Command maxShooter() {
        logActiveOverrideCommands("maxShooter", "stopShooter");
        return Commands.sequence(
            m_shooter.setFlywheelVelocityCmd(kFlywheelMaxRPS)
        );
    }

    public Command stopShooter() {
        logActiveOverrideCommands("stopShooter", "maxShooter");
        return Commands.sequence(
            m_shooter.setFlywheelVelocityCmd(kFlywheelZeroRPS)
        );
    }

    public Command turret180() {
        logActiveOverrideCommands("turret180", "turret0");
        return Commands.sequence(
            m_shooter.setTurretPositionCmd(Angle.ofBaseUnits(180, Degree))
        );
    }

    public Command turret0() {
        logActiveOverrideCommands("turret0", "turret180");
        return Commands.sequence(
            m_shooter.setTurretPositionCmd(Angle.ofBaseUnits(0, Degree))
        );
    }

    public Command hood30() {
        logActiveOverrideCommands("hood30", "hood0");
        return Commands.sequence(
            m_shooter.setHoodPositionCmd(Angle.ofBaseUnits(30, Degree))
        );
    }

    public Command hood0() {
        logActiveOverrideCommands("hood0", "hood30");
        return Commands.sequence(
            m_shooter.setHoodPositionCmd(Angle.ofBaseUnits(0, Degree))
        );
    }

    public Command startSpinner() {
        logActiveOverrideCommands("startSpinner", "stopSpinner");
        return Commands.sequence(
            m_indexer.startSpinner()
        );
    }

    public Command stopSpinner() {
        logActiveOverrideCommands("stopSpinner", "startSpinner");
        return Commands.sequence(
            m_indexer.stopSpinner()
        );
    }

    public Command startExhaust() {
        logActiveOverrideCommands("startExhaust", "stopExhaust");
        return Commands.sequence(
            m_indexer.startExhaust()
        );
    }

    public Command stopExhaust() {
        logActiveOverrideCommands("stopExhaust", "startExhaust");
        return Commands.sequence(
            m_indexer.stopExhaust()
        );
    }

    public Command maxRollers() {
        logActiveOverrideCommands("maxRollers", "stopRollers");
        return Commands.sequence(
            m_intake.setRollersSpeed(RollersVelocity.MAX)
        );
    }

    public Command stopRollers() {
        logActiveOverrideCommands("stopRollers", "maxRollers");
        return Commands.sequence(
            m_intake.setRollersSpeed(RollersVelocity.STOP)
        );
    }

    public Command deployIntake() {
        logActiveOverrideCommands("deployIntake", "safeIntake", "intakeUp");
        return Commands.sequence(
            m_intake.setDeployPos(DeployPosition.DEPLOYED)
        );
    }

    public Command safeIntake() {
        logActiveOverrideCommands("safeIntake", "deployIntake", "intakeUp");
        return Commands.sequence(
            m_intake.setDeployPos(DeployPosition.SAFE)
        );
    }

    public Command intakeUp() {
        logActiveOverrideCommands("intakeUp", "safeIntake", "deployIntake");
        return Commands.sequence(
            m_intake.setDeployPos(DeployPosition.RETRACTED)
        );
    }

    /**
     * Adds and removes specified override Command names from the activeOverridesCommands ArrayList, then logs the ArrayList.
     * @param toAdd override Command name to add.
     * @param toRemove override Command names to remove.
     */
    private void logActiveOverrideCommands(String toAdd, String... toRemove) {
        if (!activeOverrideCommands.contains(toAdd)) {
            activeOverrideCommands.add(toAdd);
        }
        for (String s : toRemove) {
            activeOverrideCommands.remove(s);
        }
        log_activeOverrideCommands.accept(activeOverrideCommands.toArray(new String[activeOverrideCommands.size()]));
    }
}