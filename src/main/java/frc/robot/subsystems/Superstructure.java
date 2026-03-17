
package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerK;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.WaltLogger;
import frc.util.WaltLogger.StringArrayLogger;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.SuperstructureK.*;
import static frc.robot.Constants.ShooterK;
import static frc.robot.Constants.IntakeK;

import java.util.HashSet;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    /* SUBSYSTEMS */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    /* LOGGERS */
    private HashSet<String> m_activeCommands = new HashSet<>();
    private final StringArrayLogger log_activeCommands = WaltLogger.logStringArray(kLogTab, "Active Commands");

    private HashSet<String> m_activeOverrideCommands = new HashSet<>();
    private final StringArrayLogger log_activeOverrideCommands = WaltLogger.logStringArray(kLogTab, "Active Override Commands");

    // private StringLogger log_shooterState = WaltLogger.logString(kLogTab, "ShooterState");
    
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
        // Command logCommand;
        // switch (pos) {
        //     case SAFE:
        //         if (m_intake.getIntakeArmStatorCurrent() < 40) {
        //             logCommand = logActiveCommands("safeIntake", "activateIntake", "retractIntake");
        //         } else {
        //             return Commands.none();
        //         }
        //         break;
        //     default:
        //         // logCommand = logActiveCommands("retractIntake", "activateIntake", "safeIntake");
        //         break;
        // }
        return Commands.sequence(
            m_intake.stopIntakeRollers(),
            m_intake.setIntakeArmPosCmd(pos),
            m_indexer.stopSpindexerCmd()
            // logCommand
        );
    }

    /**
     * 
     * @param isShooting
     * @return
     */
    public Command intake(BooleanSupplier isShooting) {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.isIntakeArmAtPos()).withTimeout(0.25),
            Commands.runEnd(
            () -> {
                boolean shooting = isShooting.getAsBoolean();

                m_shooter.setIntaking(!shooting);
                m_intake.setIntakeRollersVelocity(Constants.IntakeK.kIntakeRollersMaxRPS);
                m_indexer.setSpindexerVelocity(shooting ? IndexerK.kSpindexerShootRPS : IndexerK.kSpindexerIntakeRPS);
            }, 
            () -> {
                boolean shooting = isShooting.getAsBoolean();

                if (m_intake.getIntakeArmStatorCurrent() < 40) {
                    m_shooter.setIntaking(false);
                    m_intake.setIntakeRollersVelocity(RotationsPerSecond.zero());   //TODO: add a isNear0Vel for rollers so we don't bring to safe until rollers are low speed
                    if (!shooting) {
                        m_indexer.setSpindexerVelocity(RotationsPerSecond.zero());
                    }
                }
            })
        );
    }

    public Command startShootSequenceNOSHOOT() {
        return Commands.parallel(
            Commands.sequence(
                m_indexer.startTunnelCmd(),
                m_indexer.startSpindexerCmd()
            )
        );
    }

    /**
     * @param update message to update the SHOOTER logger
     * @return a Command that updates the logger
     */
    // public Command up(String update) {
    //     return Commands.runOnce(() -> log_shooterState.accept(update));
    // }

    /**
     * Turns on spinner and exhaust and sets shooter speed to RPS.
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command activateOuttake(Supplier<AngularVelocity> RPS) {
        // log_shooterState.accept("pre sequence");
        return Commands.sequence(
            // up("post sequence call"),
            m_shooter.setShooterVelocityCmdSupp(RPS),
            // up("post supplier command"),

            // up("pre waituntil command"),
            Commands.waitUntil(() -> m_shooter.isShooterSpunUp()),
            // up("post waituntil command"),

            // up("pre start indexer"),
            m_indexer.startIndexerCmd(),
            // up("post start indexer"),

            // up("pre repeating sequence"),
            Commands.repeatingSequence(
                // up("in repeating sequence"),
                // Commands.waitUntil(() -> !m_shooter.isShooterSpunUp()), // block until un-spun
                // m_indexer.stopIndexerCmd(), // stop indexer
                // // up("restart indexer in repeating sequence"),
                // Commands.waitUntil(() -> m_shooter.isShooterSpunUp()), // block until spun
                // m_indexer.startIndexerCmd() // restart indexer
                // // up("end repeating sequence")
                // Commands.print("cope sequence: INITIATED")
                Commands.none()
            )
            // up("post repeating sequence")
        )
        .finallyDo(
            () -> {
                deactivateOuttake();
                // log_shooterState.accept("post deactivate outtake");
            }
        );
    }

    /**
     * Turns on spinner and exhaust and sets shooter speed to CALCULATE SHOT RPS
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command activateOuttakeShotCalc() {
        // log_shooterState.accept("pre sequence");
        return Commands.parallel(
            // up("post sequence call"),
            m_shooter.shootFromCalc(),
            // up("post supplier command"),

            Commands.sequence(
                // up("pre waituntil command"),
                Commands.waitUntil(() -> m_shooter.isShooterSpunUp()),
                // up("post waituntil command"),

                // up("pre start indexer"),
                m_indexer.startIndexerCmd(),
                // up("post start indexer"),

                // up("pre repeating sequence"),
                Commands.repeatingSequence(
                    // up("in repeating sequence"),
                    // m_indexer.stopIndexerCmd()
                    //     .onlyIf(() -> !m_shooter.isShooterSpunUp())
                    //     .andThen(m_indexer.startIndexerCmd()).beforeStarting(Commands.waitUntil(() -> m_shooter.isShooterSpunUp()))
                    // up("end repeating sequence")
                    // Commands.print("shotCalc cope sequence: INITIATED")
                    Commands.none()
                )
            )
            // up("post repeating sequence")
        )
        .finallyDo(
            () -> {
                deactivateOuttake();
                // log_shooterState.accept("post deactivate outtake");
            }
        );
    }

    public Command activateOuttakeNOSHOOT() {
        return Commands.parallel(
            startShootSequenceNOSHOOT()
        ).finallyDo(
            () -> deactivateOuttakeNOSHOOT()
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
        // m_shooter.setHoodPosition(Degrees.of(1));

        // Commands.sequence(logActiveCommands("deactivateOuttake", "shooting", "emergencyDump"));
    }

    public void deactivateOuttakeNOSHOOT() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        // m_shooter.setHoodPosition(Degrees.of(1));

        // Commands.sequence(logActiveCommands("deactivateOuttake", "shooting", "emergencyDump"));
    }

    public Command emergencyBarf() {
        return Commands.startEnd(
            () -> {
                m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
                // m_shooter.setHoodPosition(ShooterK.kHoodMaxDegs);
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPS);
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPS);
                m_shooter.setShooterVelocity(ShooterK.kShooterBarfRPS);
                m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersMaxRPS.unaryMinus());
            },
            () -> {
                m_intake.setIntakeRollersVelocity(RotationsPerSecond.zero());
                m_intake.setIntakeArmPos(IntakeArmPosition.SAFE);
                m_shooter.setShooterVelocity(RotationsPerSecond.zero());
                // m_shooter.setHoodPosition(ShooterK.kHoodSafeDegs);
                m_indexer.setSpindexerVelocity(RotationsPerSecond.zero());
                m_indexer.setTunnelVelocity(RotationsPerSecond.zero());
            }
        );
    }

    public Command shimmy() {
       return m_intake.shimmy();
    }

    /**
     * Rotates the turret to the given degs
     * @param degs degrees to rotate to.
     */
    public Command turretTo(Angle degs) {
        // Command logCommand;
        // if (degs.magnitude() == 180) {
        //     logCommand = logActiveOverrideCommands("turret180", "turret0");
        // } else {
        //     logCommand = logActiveOverrideCommands("turret0", "turret180");
        // }
        return Commands.sequence(
            m_shooter.setTurretPosCmd(Rotations.of(degs.in(Rotations)))
            // logCommand
        );
    }

    /**
     * Raises/lowers the hood to the given degs.
     * @param degs degrees to raise/lower to.
     * @return
     */
    public Command hoodTo(Angle degs) {
        return Commands.sequence(
            // m_shooter.setHoodPositionCmd(degs)
        );
    }

    /**
     * Starts the indexer spinner.
     */
    public Command startSpindexerCmd() {
        return Commands.sequence(
            m_indexer.startSpindexerCmd()
            // logActiveOverrideCommands("startSpindexerCmd", "stopSpindexerCmd")
        );
    }

    /**
     * Stops the indexer spinner.
     */
    public Command stopSpindexerCmd() {
        return Commands.sequence(
            m_indexer.stopSpindexerCmd()
            // logActiveOverrideCommands("stopSpindexerCmd", "startSpindexerCmd")
        );
    }

    public Command unjamCmd(BooleanSupplier isShooting) {
        return Commands.runEnd(
            () -> {
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPS.unaryMinus());
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPS.unaryMinus());
                if (!isShooting.getAsBoolean()) {
                    m_shooter.setShooterVelocity(ShooterK.kShooterRPS.unaryMinus());
                }
            }, () -> {
                if (!isShooting.getAsBoolean()) {
                    m_shooter.setShooterVelocity(RotationsPerSecond.zero());
                    m_indexer.setSpindexerVelocity(RotationsPerSecond.zero());
                    m_indexer.setTunnelVelocity(RotationsPerSecond.zero());
                }
                else if (isShooting.getAsBoolean()) {
                    m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPS);
                    m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPS);
                }
            }
        );
    }

    /**
     * Starts the indexer exhaust.
     */
    public Command startTunnelCmd() {
        return Commands.sequence(
            m_indexer.startTunnelCmd()
            // logActiveOverrideCommands("startTunnelCmd", "stopTunnelCmd")
        );
    }

    /**
     * Stops the indexer exhaust.
     */
    public Command stopTunnelCmd() {
        return Commands.sequence(
            m_indexer.stopTunnelCmd()
            // logActiveOverrideCommands("stopTunnelCmd", "startTunnelCmd")
        );
    }

    /**
     * Starts the intake rollers.
     */
    public Command startIntakeRollers() {
        return Commands.sequence(
            m_intake.startIntakeRollers()
            // logActiveOverrideCommands("startIntakeRollers", "stopIntakeRollers")
        );
    }

    /**
     * Stops the intake rollers.
     */
    public Command stopIntakeRollers() {
        return Commands.sequence(
            m_intake.stopIntakeRollers()
            // logActiveOverrideCommands("stopIntakeRollers", "startIntakeRollers")
        );
    }

    /**
     * Deploys the intake to the given pos.
     * @param pos position to deploy to.
     * @return
     */
    public Command intakeTo(IntakeArmPosition pos) {
        // Command logCommand;
        // switch (pos) {
        //     case DEPLOYED:
        //         logCommand = logActiveOverrideCommands("deployIntake", "safeIntake", "intakeUp");
        //         break;
        //     case SAFE:
        //         logCommand = logActiveOverrideCommands("safeIntake", "deployIntake", "intakeUp");
        //         break;
        //     default:
        //         if (m_intake.getIntakeArmStatorCurrent() < 40) {
        //             logCommand = logActiveOverrideCommands("intakeUp", "safeIntake", "deployIntake");
        //         }
        //         else {
        //            return Commands.none();
        //         }
        //         break;
        // }
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(pos)
            // logCommand
        );
    }

    // /**
    //  * Adds and removes specified Command names from the ActiveCommands ArrayList, then logs the ArrayList.
    //  * @param toAdd Command name to add.
    //  * @param toRemove Command names to remove.
    //  */
    // private Command logActiveCommands(String toAdd, String... toRemove) {
    //     Command addTo = Commands.runOnce(
    //         () -> m_activeCommands.add(toAdd)
    //     );
    //     Command removeFrom = Commands.runOnce(
    //         () -> {
    //             for (String s : toRemove) {
    //                 m_activeCommands.remove(s);
    //             }
    //         }
    //     );
    //     Command updateLog = Commands.runOnce(
    //         () -> log_activeCommands.accept(m_activeCommands.toArray(new String[m_activeCommands.size()]))
    //     );

    //     return Commands.sequence(
    //         addTo,
    //         removeFrom,
    //         updateLog
    //     );
    // }

    // /**
    //  * Adds and removes specified override Command names from the activeOverridesCommands ArrayList, then logs the ArrayList.
    //  * @param toAdd override Command name to add.
    //  * @param toRemove override Command names to remove.
    //  */
    // private Command logActiveOverrideCommands(String toAdd, String... toRemove) {
    //     Command addTo = Commands.runOnce(
    //         () -> m_activeOverrideCommands.add(toAdd)   
    //     );
    //     Command removeFrom = Commands.runOnce(
    //         () -> {
    //             for (String s : toRemove) {
    //                 m_activeOverrideCommands.remove(s);
    //             }
    //         }
    //     );
    //     Command updateLog = Commands.runOnce(
    //         () -> log_activeOverrideCommands.accept(m_activeOverrideCommands.toArray(new String[m_activeOverrideCommands.size()]))
    //     );

    //     return Commands.sequence(
    //         addTo,
    //         removeFrom,
    //         updateLog
    //     );
    // }
}
