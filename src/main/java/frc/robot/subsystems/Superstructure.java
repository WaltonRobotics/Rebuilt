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
import static frc.robot.Constants.ShooterK.kFlywheelMaxRPS;
import static frc.robot.Constants.ShooterK.kFlywheelZeroRPS;

import java.util.HashSet;

public class Superstructure {

    /* declare subsystems */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    /* loggers */
    private HashSet<String> m_activeCommands = new HashSet<>();
    private final StringArrayLogger log_activeCommands = WaltLogger.logStringArray(kLogTab, "Active Commands");

    private HashSet<String> m_activeOverrideCommands = new HashSet<>();
    private final StringArrayLogger log_activeOverrideCommands = WaltLogger.logStringArray(kLogTab, "Active Override Commands");
    
    /* constructor */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* button bind sequences */

    // Regular Commands
    /**
     * Turns off rollers and spinner and moves deploy to given pos.
     * @param pos the position to move deploy to.
     */
    public Command deactivateIntake(DeployPosition pos) {
        Command logCommand;
        switch (pos) {
            case SAFE:
                if (m_intake.getDeployMotor().getStatorCurrent().getValueAsDouble() < 40) {
                    logCommand = logActiveCommands("safeIntake", "activateIntake", "retractIntake");
                } else {
                    return Commands.none();
                }
                break;
            default:
                logCommand = logActiveCommands("retractIntake", "activateIntake", "safeIntake");
                break;
        }
        return Commands.sequence(
            m_indexer.stopSpinner(),
            m_intake.setDeployPos(pos),
            m_intake.setRollersSpeed(RollersVelocity.STOP),
            logCommand
        );
    }

    /**
     * Turns on rollers and spinner moves deploy to deployed positon.
     */
    public Command activateIntake() {
        return Commands.sequence(
            m_intake.setRollersSpeed(RollersVelocity.MAX),
            m_indexer.startSpinner(),
            m_intake.setDeployPos(DeployPosition.DEPLOYED),
            logActiveCommands("activateIntake", "safeIntake", "retractIntake")
        );
    }

    /**
     * Turns on spinner and exhaust and sets shooter speed to RPS.
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command activateOuttake(AngularVelocity RPS) {
        Command logCommand;
        if (RPS == kFlywheelMaxRPS) {
            logCommand = logActiveCommands("shooting", "deactivateOuttake", "emergencyDump");   
        } else {
            logCommand = logActiveCommands("emergencyDump", "shooting", "deactivateOuttake");
        }
        return Commands.sequence(
            m_indexer.startSpinner(),
            m_indexer.startExhaust(),
            m_shooter.setFlywheelVelocityCmd(RPS),
            logCommand
        );
    }

    /**
     * Turns off spinner, exhaust, and shooter.
     * <p>
     * Note: does not move turret or hood.
     */
    public Command deactivateOuttake() {
        return Commands.sequence(
            m_indexer.stopSpinner(),
            m_indexer.stopExhaust(),
            m_shooter.setFlywheelVelocityCmd(kFlywheelZeroRPS),
            logActiveCommands("deactivateOuttake", "shooting", "emergencyDump")
        );
    }
    
    /**
     * Initiates passing by activating intake and outtake.
     */
    public Command startPassing() {
        return Commands.sequence(
            activateIntake(),
            activateOuttake(kFlywheelMaxRPS),
            logActiveCommands("startPassing", "stopPassing")
        );
    }

    /**
     * Exits passing mode by deactivating intake with deploy to SAFE and deactivating outtake.
     */
    public Command stopPassing() {
        return Commands.sequence(
            deactivateIntake(DeployPosition.SAFE),
            deactivateOuttake(),
            logActiveCommands("stopPassing", "startPassing")
        );
    }

    /**
     * Adds and removes specified Command names from the ActiveCommands ArrayList, then logs the ArrayList.
     * @param toAdd Command name to add.
     * @param toRemove Command names to remove.
     */
    private Command logActiveCommands(String toAdd, String... toRemove) {
        Command addTo = Commands.runOnce(
            () -> m_activeCommands.add(toAdd)
        );
        Command removeFrom = Commands.runOnce(
            () -> {
                for (String s : toRemove) {
                    m_activeCommands.remove(s);
                }
            }
        );
        Command updateLog = Commands.runOnce(
            () -> log_activeCommands.accept(m_activeCommands.toArray(new String[m_activeCommands.size()]))
        );

        return Commands.sequence(
            addTo,
            removeFrom,
            updateLog
        );
    }

    // Override commands
    /**
     * Sets the shooter speed to max.
     */
    public Command maxShooter() {
        return Commands.sequence(
            m_shooter.setFlywheelVelocityCmd(kFlywheelMaxRPS),
            logActiveOverrideCommands("maxShooter", "stopShooter")
        );
    }

    /**
     * Stops the shooter.
     */
    public Command stopShooter() {
        return Commands.sequence(
            m_shooter.setFlywheelVelocityCmd(kFlywheelZeroRPS),
            logActiveOverrideCommands("stopShooter", "maxShooter")
        );
    }

    /**
     * Rotates the turret to the given degs
     * @param degs degrees to rotate to.
     */
    public Command turretTo(double degs) {
        Command logCommand;
        if (degs == 180) {
            logCommand = logActiveOverrideCommands("turret180", "turret0");
        } else {
            logCommand = logActiveOverrideCommands("turret0", "turret180");
        }
        return Commands.sequence(
            m_shooter.setTurretPositionCmd(Angle.ofBaseUnits(degs, Degree)),
            logCommand
        );
    }

    /**
     * Raises/lowers the hood to the given degs.
     * @param degs degrees to raise/lower to.
     * @return
     */
    public Command hoodTo(double degs) {
        Command logCommand;
        if (degs == 30) {
            logCommand = logActiveOverrideCommands("hood30", "hood0");
        } else {
            logCommand = logActiveOverrideCommands("hood0", "hood30");
        }
        return Commands.sequence(
            m_shooter.setHoodPositionCmd(Angle.ofBaseUnits(degs, Degree)),
            logCommand
        );
    }

    /**
     * Starts the indexer spinner.
     */
    public Command startSpinner() {
        return Commands.sequence(
            m_indexer.startSpinner(),
            logActiveOverrideCommands("startSpinner", "stopSpinner")
        );
    }

    /**
     * Stops the indexer spinner.
     */
    public Command stopSpinner() {
        return Commands.sequence(
            m_indexer.stopSpinner(),
            logActiveOverrideCommands("stopSpinner", "startSpinner")
        );
    }

    /**
     * Starts the indexer exhaust.
     */
    public Command startExhaust() {
        return Commands.sequence(
            m_indexer.startExhaust(),
            logActiveOverrideCommands("startExhaust", "stopExhaust")
        );
    }

    /**
     * Stops the indexer exhaust.
     */
    public Command stopExhaust() {
        return Commands.sequence(
            m_indexer.stopExhaust(),
            logActiveOverrideCommands("stopExhaust", "startExhaust")
        );
    }

    /**
     * Sets the intake rollers speed to the given RPS
     * @param RPS RPS to set speed to.
     * @return
     */
    public Command setRollersSpeed(RollersVelocity RPS) {
        Command logCommand;
        switch (RPS) {
            case MAX:
                logCommand = logActiveOverrideCommands("maxRollers", "stopRollers");
                break;
            default:
                logCommand = logActiveOverrideCommands("stopRollers", "maxRollers");
                break;
        }
        return Commands.sequence(
            m_intake.setRollersSpeed(RPS),
            logCommand
        );
    }

    /**
     * Deploys the intake to the given pos.
     * @param pos position to deploy to.
     * @return
     */
    public Command intakeTo(DeployPosition pos) {
        Command logCommand;
        switch (pos) {
            case DEPLOYED:
                logCommand = logActiveOverrideCommands("deployIntake", "safeIntake", "intakeUp");
                break;
            case SAFE:
                logCommand = logActiveOverrideCommands("safeIntake", "deployIntake", "intakeUp");
                break;
            default:
                if (m_intake.getDeployMotor().getStatorCurrent().getValueAsDouble() < 40) {
                    logCommand = logActiveOverrideCommands("intakeUp", "safeIntake", "deployIntake");
                }
                else {
                   return Commands.none();
                }
                break;
        }
        return logCommand = Commands.sequence(
            m_intake.setDeployPos(pos),
            logCommand
        );
    }

    /**
     * Adds and removes specified override Command names from the activeOverridesCommands ArrayList, then logs the ArrayList.
     * @param toAdd override Command name to add.
     * @param toRemove override Command names to remove.
     */
    private Command logActiveOverrideCommands(String toAdd, String... toRemove) {
        Command addTo = Commands.runOnce(
            () -> m_activeOverrideCommands.add(toAdd)   
        );
        Command removeFrom = Commands.runOnce(
            () -> {
                for (String s : toRemove) {
                    m_activeOverrideCommands.remove(s);
                }
            }
        );
        Command updateLog = Commands.runOnce(
            () -> log_activeOverrideCommands.accept(m_activeOverrideCommands.toArray(new String[m_activeOverrideCommands.size()]))
        );

        return Commands.sequence(
            addTo,
            removeFrom,
            updateLog
        );
    }
}