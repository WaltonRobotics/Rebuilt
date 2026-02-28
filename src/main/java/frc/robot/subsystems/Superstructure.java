package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerK;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.WaltLogger;
import frc.util.WaltLogger.StringArrayLogger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.SuperstructureK.*;
import static frc.robot.Constants.ShooterK;
import static frc.robot.Constants.IntakeK;

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
            m_intake.stopIntakeRollers(),
            m_intake.setIntakeArmPosCmd(pos),
            m_indexer.stopSpindexerCmd(),
            logCommand
        );
    }

    /**
     * Turns on rollers and spinner moves deploy to deployed positon.
     */
    public Command activateIntake() {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.isIntakeArmAtPos()),
            m_intake.startIntakeRollers(),
            logActiveCommands("activateIntake", "safeIntake", "retractIntake")
        );
    }

    public Command startShootSequence(AngularVelocity RPS, Angle hoodDegs) {
        return Commands.sequence(
            m_shooter.setShooterVelocityCmd(RPS),
            m_shooter.setHoodPositionCmd(hoodDegs),
            Commands.waitUntil(() -> m_shooter.isShooterSpunUp()).withTimeout(3),
            m_indexer.startTunnelCmd(),
            m_indexer.startSpindexerCmd()
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

        return Commands.parallel(
            startShootSequence(RPS, Degrees.of(20))
                .onlyWhile(() -> m_shooter.isShooterSpunUp())
                .andThen(Commands.waitUntil(() -> m_shooter.isShooterSpunUp()))
                .repeatedly(),
            m_intake.shimmy(),
            logCommand
        ).finallyDo(
            () -> deactivateOuttake()
        );
    }

    /**
     * Turns off spinner, exhaust, and shooter.
     * <p>
     * Note: does not move turret or hood.
     */
    public void deactivateOuttake() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
        m_shooter.setHoodPosition(Degrees.of(1));

        Commands.sequence(logActiveCommands("deactivateOuttake", "shooting", "emergencyDump"));
    }

    public Command emergencyBarf() {
        return Commands.startEnd(
            () -> {
                m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
                m_shooter.setHoodPosition(ShooterK.kHoodMaxDegs);
                m_indexer.setTunnelVelocity(IndexerK.m_tunnelRPS);
                m_indexer.setSpindexerVelocity(IndexerK.m_spindexerRPS);
                m_shooter.setShooterVelocity(ShooterK.kShooterBarfRPS);
                m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersMaxRPS.times(-1));
            },
            () -> {
                m_intake.setIntakeRollersVelocity(RotationsPerSecond.of(0));
                m_intake.setIntakeArmPos(IntakeArmPosition.SAFE);
                m_shooter.setShooterVelocity(RotationsPerSecond.of(0));
                m_shooter.setHoodPosition(ShooterK.kHoodSafeDegs);
                m_indexer.setSpindexerVelocity(RotationsPerSecond.of(0));
                m_indexer.setTunnelVelocity(RotationsPerSecond.of(0));
            }
        );
    }

    public Command shimmy() {
       return m_intake.shimmy();
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
    // public Command stopPassing() {
    //     return Commands.sequence(
    //         deactivateIntake(IntakeArmPosition.SAFE),
    //         deactivateOuttake(),
    //         logActiveCommands("stopPassing", "startPassing")
    //     );
    // }

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
    public Command startSpindexerCmd() {
        return Commands.sequence(
            m_indexer.startSpindexerCmd(),
            logActiveOverrideCommands("startSpindexerCmd", "stopSpindexerCmd")
        );
    }

    /**
     * Stops the indexer spinner.
     */
    public Command stopSpindexerCmd() {
        return Commands.sequence(
            m_indexer.stopSpindexerCmd(),
            logActiveOverrideCommands("stopSpindexerCmd", "startSpindexerCmd")
        );
    }

    /**
     * Starts the indexer exhaust.
     */
    public Command startTunnelCmd() {
        return Commands.sequence(
            m_indexer.startTunnelCmd(),
            logActiveOverrideCommands("startTunnelCmd", "stopTunnelCmd")
        );
    }

    /**
     * Stops the indexer exhaust.
     */
    public Command stopTunnelCmd() {
        return Commands.sequence(
            m_indexer.stopTunnelCmd(),
            logActiveOverrideCommands("stopTunnelCmd", "startTunnelCmd")
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
            m_intake.setIntakeArmPosCmd(pos),
            logCommand
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