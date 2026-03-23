
package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerK;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.SuperstructureK;
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
    private final Swerve m_drivetrain;

    /* LOGGERS */
    private HashSet<String> m_activeCommands = new HashSet<>();
    private final StringArrayLogger log_activeCommands = WaltLogger.logStringArray(kLogTab, "Active Commands");

    private HashSet<String> m_activeOverrideCommands = new HashSet<>();
    private final StringArrayLogger log_activeOverrideCommands = WaltLogger.logStringArray(kLogTab, "Active Override Commands");

    // private StringLogger log_shooterState = WaltLogger.logString(kLogTab, "ShooterState");
    
    /* CONSTRUCTOR */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter, Swerve drivetrain) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
        m_drivetrain = drivetrain;
    }

    /* BUTTON BIND SEQUENCES */

    /**
     * 
     * @param isShooting
     * @return
     */
    public Command intake(BooleanSupplier isShooting) {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.isIntakeArmAtPos()).withTimeout(0.25),
            Commands.run(
            () -> {
                boolean shooting = isShooting.getAsBoolean();

                m_shooter.setIntaking(!shooting);
                m_intake.setIntakeRollersVelocity(12);
                m_indexer.setSpindexerVelocity(shooting ? IndexerK.kSpindexerShootRPS : IndexerK.kSpindexerIntakeRPS);
            })
        ).finallyDo(
            () -> {
                boolean shooting = isShooting.getAsBoolean();
                m_shooter.setIntaking(false);
                m_intake.setIntakeRollersVelocity(0);
                if (!shooting) {
                    m_indexer.setSpindexerVelocity(RotationsPerSecond.zero());
                }
            }
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
        return Commands.parallel(
            // up("post sequence call"),
            m_shooter.setShooterVelocityCmdSupp(RPS),
            // up("post supplier command"),

            Commands.sequence(
                // up("pre waituntil command"),
                Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocity().gte(ShooterK.kShooterSpunUpMinimum)))).withTimeout(ShooterK.kShooterSpunUpTimeout),
                m_indexer.startTunnelCmd(),
                Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocity().gte(IndexerK.kTunnelSpunUpMinimum))).withTimeout(IndexerK.kTunnelSpunUpTimeout),
                // up("post waituntil command"),

                // up("pre start indexer"),
                m_indexer.startSpindexerCmd(),
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
                Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocity().gte(ShooterK.kShooterSpunUpMinimum)))).withTimeout(ShooterK.kShooterSpunUpTimeout),
                m_indexer.startTunnelCmd(),
                Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocity().gte(IndexerK.kTunnelSpunUpMinimum))).withTimeout(IndexerK.kTunnelSpunUpTimeout),
                // up("post waituntil command"),

                // up("pre start indexer"),
                m_indexer.startSpindexerCmd()
                // up("post start indexer"),
            )
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
                m_intake.setIntakeRollersVelocity(-12);
            },
            () -> {
                m_intake.setIntakeRollersVelocity(0);
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

    /**
     * runs each subsytem in order of how the ball will flow through the robot (intake -> spindxer -> tunnel -> shooter -> swerve) 
     * and then everything runs together like they would in a match
     * 
     * @return an automated ops check that runs each subsystem individually
     */
    public Command longOpsCheck() {
        return Commands.sequence(
            Commands.print("======================STARTING LONG OPS CHECK======================"),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //deploy intake
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //run intake rollers
            m_intake.startIntakeRollers(),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //run spindexer
            m_intake.stopIntakeRollers(),
            m_indexer.setSpindexerVelocityCmd(Constants.IndexerK.kSpindexerShootRPS),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //Stop Spindexer; run tunnel
            m_indexer.stopSpindexerCmd(),
            m_indexer.setTunnelVelocityCmd(IndexerK.kTunnelShootRPS),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //Stop tunnel; run shooter
            m_indexer.stopTunnelCmd(),
            m_shooter.setShooterVelocityCmd(ShooterK.kShooterRPS),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //shooter stopped
            m_shooter.setShooterVelocityCmd(RotationsPerSecond.of(0)),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //Run short Ops check (intake, shoot, swerve)
            shortOpsCheck()
        );
    }

    /**
     * runs the intaking cmd – 5 seconds,
     * the outaking cmd – 4 seconds,
     * and then the swerve tests (run wheels forward, backward, and then rotates them) – 2.5 seconds between each test
     * 
     * @return a sequence of commands that runs the subsystems in groups – how they would run together in a match
     * (ex. intaking cmd runs intake arm and rollers, spindexer and turret)
     */
    public Command shortOpsCheck() {
        return Commands.sequence(
            //runs intaking cmd
            intake(() -> false).withTimeout(SuperstructureK.kShortOpsCheckIntakeTime),
            Commands.waitSeconds(SuperstructureK.kShortOpsCheckPause),

            //runs shooting cmd
            activateOuttakeShotCalc(),
            Commands.waitSeconds(SuperstructureK.kShortOpsCheckPause),

            //Swerve test
            m_drivetrain.swerveAutomatedOpsCheck()
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
