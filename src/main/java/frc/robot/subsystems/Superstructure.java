package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeK;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.util.WaltLogger;
import frc.util.WaltLogger.StringArrayLogger;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.SuperstructureK.*;
import static frc.robot.Constants.ShooterK;
import static frc.robot.Constants.IntakeK.kIntakeRollersMaxRPS;

import java.util.HashSet;

public class Superstructure {
    /* SUBSYSTEMS */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    /* LOGGERS */
    private HashSet<String> m_activeCommands = new HashSet<>();
    private final StringArrayLogger log_activeCommands = WaltLogger.logStringArray(kLogTab, "Active Commands");

    private HashSet<String> m_activeOverrideCommands = new HashSet<>();
    private final StringArrayLogger log_activeOverrideCommands = WaltLogger.logStringArray(kLogTab, "Active Override Commands");
    
    /* CONSTRUCTOR */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* BUTTON BIND SEQUENCES */

    /**
     * Turns off rollers and spinner and moves deploy to given pos.
     * @param pos the position to move deploy to.
     */
    public Command deactivateIntake(IntakeArmPosition pos) {
        Command logCommand;
        switch (pos) {
            case SAFE:
                if (m_intake.getIntakeArmMotor().getStatorCurrent().getValueAsDouble() < 40) {
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
            m_indexer.stopSpindexer(),
            m_intake.setIntakeArmPos(pos),
            m_intake.stopIntakeRollers(),
            logCommand
        );
    }

    /**
     * Turns on rollers and spinner moves deploy to deployed positon.
     */
    public Command activateIntake() {
        return Commands.sequence(
            m_intake.startIntakeRollers(),
            m_indexer.startSpindexer(),
            m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED),
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
        if (RPS == ShooterK.kShooterMaxRPS) {
            logCommand = logActiveCommands("shooting", "deactivateOuttake", "emergencyDump");   
        } else {
            logCommand = logActiveCommands("emergencyDump", "shooting", "deactivateOuttake");
        }

        return Commands.sequence(
            m_shooter.setShooterVelocityCmd(RPS),
            Commands.waitUntil(() -> m_shooter.checkIfSpunUp(RPS.magnitude())),
            m_indexer.startTunnel(),
            m_indexer.startSpindexer(),
            // m_intake.setIntakeRollersVelocityCmd(RotationsPerSecond.of((0.25) * IntakeK.kIntakeRollersMaxRPS.magnitude())),
            logCommand
        );
    }

    public Command shimmy() {
        return Commands.sequence(
            m_intake.setIntakeRollersVelocityCmd(RotationsPerSecond.of(IntakeK.kIntakeRollersMaxRPS.magnitude() * (0.25))),
            m_intake.setIntakeArmPos(IntakeArmPosition.RETRACTED),
            Commands.waitUntil(() -> m_intake.intakeArmAtPos(IntakeArmPosition.RETRACTED)),
            m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.intakeArmAtPos(IntakeArmPosition.DEPLOYED))
        );
    }

    /**
     * Turns off spinner, exhaust, and shooter.
     * <p>
     * Note: does not move turret or hood.
     */
    public Command deactivateOuttake() {
        return Commands.sequence(
            m_indexer.stopSpindexer(),
            m_indexer.stopTunnel(),
            m_shooter.setShooterVelocityCmd(ShooterK.kShooterZeroRPS),
            logActiveCommands("deactivateOuttake", "shooting", "emergencyDump")
        );
    }
    
    /**
     * Initiates passing by activating intake and outtake.
     */
    public Command startPassing() {
        return Commands.sequence(
            activateIntake(),
            activateOuttake(ShooterK.kShooterMaxRPS),
            logActiveCommands("startPassing", "stopPassing")
        );
    }

    /**
     * Exits passing mode by deactivating intake with deploy to SAFE and deactivating outtake.
     */
    public Command stopPassing() {
        return Commands.sequence(
            deactivateIntake(IntakeArmPosition.SAFE),
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
            m_shooter.setShooterVelocityCmd(ShooterK.kShooterMaxRPS),
            // m_shooter.setShooterVelocityCmd(RotationsPerSecond.of(50)),
            logActiveOverrideCommands("maxShooter", "stopShooter")
        );
    }

    /**
     * Stops the shooter.
     */
    public Command stopShooter() {
        return Commands.sequence(
            m_shooter.setShooterVelocityCmd(ShooterK.kShooterZeroRPS),
            logActiveOverrideCommands("stopShooter", "maxShooter")
        );
    }

    /**
     * Rotates the turret to the given degs
     * @param degs degrees to rotate to.
     */
    public Command turretTo(Angle degs) {
        Command logCommand;
        if (degs.magnitude() == 180) {
            logCommand = logActiveOverrideCommands("turret180", "turret0");
        } else {
            logCommand = logActiveOverrideCommands("turret0", "turret180");
        }
        return Commands.sequence(
            m_shooter.setTurretPositionCmd(Rotations.of(degs.in(Rotations))),
            logCommand
        );
    }

    /**
     * Raises/lowers the hood to the given degs.
     * @param degs degrees to raise/lower to.
     * @return
     */
    public Command hoodTo(Angle degs) {
        return Commands.sequence(
            m_shooter.setHoodPositionCmd(degs)
        );
    }

    /**
     * Starts the indexer spinner.
     */
    public Command startSpindexer() {
        return Commands.sequence(
            m_indexer.startSpindexer(),
            logActiveOverrideCommands("startSpindexer", "stopSpindexer")
        );
    }

    /**
     * Stops the indexer spinner.
     */
    public Command stopSpindexer() {
        return Commands.sequence(
            m_indexer.stopSpindexer(),
            logActiveOverrideCommands("stopSpindexer", "startSpindexer")
        );
    }

    /**
     * Starts the indexer exhaust.
     */
    public Command startTunnel() {
        return Commands.sequence(
            m_indexer.startTunnel(),
            logActiveOverrideCommands("startTunnel", "stopTunnel")
        );
    }

    /**
     * Stops the indexer exhaust.
     */
    public Command stopTunnel() {
        return Commands.sequence(
            m_indexer.stopTunnel(),
            logActiveOverrideCommands("stopTunnel", "startTunnel")
        );
    }

    /**
     * Starts the intake rollers.
     */
    public Command startIntakeRollers() {
        return Commands.sequence(
            m_intake.startIntakeRollers(),
            logActiveOverrideCommands("startIntakeRollers", "stopIntakeRollers")
        );
    }

    /**
     * Stops the intake rollers.
     */
    public Command stopIntakeRollers() {
        return Commands.sequence(
            m_intake.stopIntakeRollers(),
            logActiveOverrideCommands("stopIntakeRollers", "startIntakeRollers")
        );
    }

    /**
     * Deploys the intake to the given pos.
     * @param pos position to deploy to.
     * @return
     */
    public Command intakeTo(IntakeArmPosition pos) {
        Command logCommand;
        switch (pos) {
            case DEPLOYED:
                logCommand = logActiveOverrideCommands("deployIntake", "safeIntake", "intakeUp");
                break;
            case SAFE:
                logCommand = logActiveOverrideCommands("safeIntake", "deployIntake", "intakeUp");
                break;
            default:
                if (m_intake.getIntakeArmMotor().getStatorCurrent().getValueAsDouble() < 40) {
                    logCommand = logActiveOverrideCommands("intakeUp", "safeIntake", "deployIntake");
                }
                else {
                   return Commands.none();
                }
                break;
        }
        return logCommand = Commands.sequence(
            m_intake.setIntakeArmPos(pos),
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