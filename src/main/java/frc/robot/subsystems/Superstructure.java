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
    private HashSet<String> activeCommands = new HashSet<>();
    private final StringArrayLogger log_activeCommands = WaltLogger.logStringArray(kLogTab, "Active Commands");

    private HashSet<String> activeOverrideCommands = new HashSet<>();
    private final StringArrayLogger log_activeOverrideCommands = WaltLogger.logStringArray(kLogTab, "Active Override Commands");
    
    /* constructor */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* button bind sequences */

    // Regular Commands
    public Command deactivateIntake(DeployPosition pos) {
        if (pos == DeployPosition.SAFE) {
            return Commands.sequence(
                m_indexer.stopSpinner(),
                m_intake.setDeployPos(pos),
                m_intake.setRollersSpeed(RollersVelocity.STOP),
                logActiveCommands("prepIntake", "activateIntake", "retractIntake")
            );
        } else {
            return Commands.sequence(
                m_indexer.stopSpinner(),
                m_intake.setDeployPos(pos),
                m_intake.setRollersSpeed(RollersVelocity.STOP),
                logActiveCommands("retractIntake", "activateIntake", "prepIntake")
            );
        }
        
    }

    public Command activateIntake() {
        return Commands.sequence(
            m_intake.setRollersSpeed(RollersVelocity.MAX),
            m_indexer.startSpinner(),
            m_intake.setDeployPos(DeployPosition.DEPLOYED),
            logActiveCommands("activateIntake", "prepIntake", "retractIntake")
        );
    }

    public Command activateOuttake(AngularVelocity rps) {
        if (rps == kFlywheelMaxRPS) {
            return Commands.sequence(
                m_indexer.startSpinner(),
                m_indexer.startExhaust(),
                m_shooter.setFlywheelVelocityCmd(rps),
                logActiveCommands("normalOuttake", "deactivateOuttake", "emergencyOuttake")
            );
            
        } else {
            return Commands.sequence(
                m_indexer.startSpinner(),
                m_indexer.startExhaust(),
                m_shooter.setFlywheelVelocityCmd(rps),
                logActiveCommands("emergencyOuttake", "normalOuttake", "deactivateOuttake")
            );
        }
    }

    public Command deactivateOuttake() {
        return Commands.sequence(
            m_indexer.stopSpinner(),
            m_indexer.stopExhaust(),
            m_shooter.setFlywheelVelocityCmd(kFlywheelZeroRPS),
            logActiveCommands("deactivateOuttake", "normalOuttake", "emergencyOuttake")
        );
    }
    
    public Command startPassing() {
        return Commands.sequence(
            activateIntake(),
            activateOuttake(kFlywheelMaxRPS),
            logActiveCommands("startPassing", "stopPassing")
        );
    }

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
            () -> activeCommands.add(toAdd)
        );
        Command removeFrom = Commands.runOnce(
            () -> {
                for (String s : toRemove) {
                    activeCommands.remove(s);
                }
            }
        );
        Command updateLog = Commands.runOnce(
            () -> log_activeCommands.accept(activeCommands.toArray(new String[activeCommands.size()]))
        );

        return Commands.sequence(
            addTo,
            removeFrom,
            updateLog
        );
    }

    // Override commands
    public Command maxShooter() {
        return Commands.sequence(
            m_shooter.setFlywheelVelocityCmd(kFlywheelMaxRPS),
            logActiveOverrideCommands("maxShooter", "stopShooter")
        );
    }

    public Command stopShooter() {
        return Commands.sequence(
            m_shooter.setFlywheelVelocityCmd(kFlywheelZeroRPS),
            logActiveOverrideCommands("stopShooter", "maxShooter")
        );
    }

    public Command turretTo(double degs) {
        if (degs == 180) {
            return Commands.sequence(
                m_shooter.setTurretPositionCmd(Angle.ofBaseUnits(degs, Degree)),
                logActiveOverrideCommands("turret180", "turret0")
            );
        } else {
            return Commands.sequence(
                m_shooter.setTurretPositionCmd(Angle.ofBaseUnits(degs, Degree)),
                logActiveOverrideCommands("turret0", "turret180")
            );
        }
    }

    public Command hoodTo(double degs) {
        if (degs == 30) {
            return Commands.sequence(
                m_shooter.setHoodPositionCmd(Angle.ofBaseUnits(degs, Degree)),
                logActiveOverrideCommands("hood30", "hood0")
            );
        } else {
            return Commands.sequence(
                m_shooter.setHoodPositionCmd(Angle.ofBaseUnits(degs, Degree)),
                logActiveOverrideCommands("hood0", "hood30")
            );
        }
        
    }

    public Command startSpinner() {
        return Commands.sequence(
            m_indexer.startSpinner(),
            logActiveOverrideCommands("startSpinner", "stopSpinner")
        );
    }

    public Command stopSpinner() {

        return Commands.sequence(
            m_indexer.stopSpinner(),
            logActiveOverrideCommands("stopSpinner", "startSpinner")
        );
    }

    public Command startExhaust() {
        return Commands.sequence(
            m_indexer.startExhaust(),
            logActiveOverrideCommands("startExhaust", "stopExhaust")
        );
    }

    public Command stopExhaust() {
        return Commands.sequence(
            m_indexer.stopExhaust(),
            logActiveOverrideCommands("stopExhaust", "startExhaust")
        );
    }

    public Command setRollersSpeed(RollersVelocity velo) {
        if (velo == RollersVelocity.MAX) {
            return Commands.sequence(
                m_intake.setRollersSpeed(velo),
                logActiveOverrideCommands("maxRollers", "stopRollers")
            );
        } else {
            return Commands.sequence(
                m_intake.setRollersSpeed(velo),
                logActiveOverrideCommands("stopRollers", "maxRollers")
            );            
        }
    }

    public Command intakeTo(DeployPosition pos) {
        if (pos == DeployPosition.DEPLOYED) {
            return Commands.sequence(
                m_intake.setDeployPos(pos),
                logActiveOverrideCommands("deployIntake", "safeIntake", "intakeUp")
            );
        } else if (pos == DeployPosition.SAFE) {
            return Commands.sequence(
                m_intake.setDeployPos(pos),
                logActiveOverrideCommands("safeIntake", "deployIntake", "intakeUp")
            );
        } else {
            return Commands.sequence(
                m_intake.setDeployPos(pos),
                logActiveOverrideCommands("intakeUp", "safeIntake", "deployIntake")
            );
        }
    }

    /**
     * Adds and removes specified override Command names from the activeOverridesCommands ArrayList, then logs the ArrayList.
     * @param toAdd override Command name to add.
     * @param toRemove override Command names to remove.
     */
    private Command logActiveOverrideCommands(String toAdd, String... toRemove) {
        Command addTo = Commands.runOnce(
            () -> activeOverrideCommands.add(toAdd)   
        );
        Command removeFrom = Commands.runOnce(
            () -> {
                for (String s : toRemove) {
                    activeOverrideCommands.remove(s);
                }
            }
        );
        Command updateLog = Commands.runOnce(
            () -> log_activeOverrideCommands.accept(activeOverrideCommands.toArray(new String[activeOverrideCommands.size()]))
        );

        return Commands.sequence(
            addTo,
            removeFrom,
            updateLog
        );
    }
}