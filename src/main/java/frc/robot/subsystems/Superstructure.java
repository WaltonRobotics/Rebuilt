
package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IndexerK;
import frc.robot.Constants.ShooterK;
import frc.robot.Constants.SuperstructureK;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.shooter.Shooter;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    /* SUBSYSTEMS */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;
    private final Swerve m_drivetrain;

    public boolean m_isLongOpsCheck = true;
    
    /* CONSTRUCTOR */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter, Swerve drivetrain) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
        m_drivetrain = drivetrain;
    }

    /* BUTTON BIND SEQUENCES */
    /**
     * @param isShooting is if the robot is shooting
     * @return the intake Command
     */
    public Command intake(BooleanSupplier isShooting) {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.isIntakeArmAtDest()).withTimeout(0.25),
            m_intake.setIntakeRollersVelocityCmd(12),
            Commands.run(
            () -> {
                boolean shooting = isShooting.getAsBoolean();
                m_shooter.m_turret.setIntaking(!shooting);
                m_indexer.setSpindexerVelocity(shooting ? IndexerK.kSpindexerShootRPS : IndexerK.kSpindexerIntakeRPS);
            })
        ).finallyDo(
            () -> {
                boolean shooting = isShooting.getAsBoolean();
                m_shooter.m_turret.setIntaking(false);
                m_intake.setIntakeRollersVelocity(0);
                if (!shooting) {
                    m_indexer.setSpindexerVelocity(RotationsPerSecond.zero());
                }
            }
        );
    }

    /**
     * Turns on spinner and exhaust and sets shooter speed to RPS.
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command activateOuttake(Supplier<AngularVelocity> RPS) {
        return Commands.parallel(
            m_shooter.setShooterVelocityCmdSupp(RPS),

            Commands.sequence(
                Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocityRotPerSec() >= ShooterK.kShooterSpunUpMinimumD))).withTimeout(ShooterK.kShooterSpunUpTimeout),
                m_indexer.startTunnelCmd(),
                Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocityRotPerSec() >= IndexerK.kTunnelSpunUpMinimumD)).withTimeout(IndexerK.kTunnelSpunUpTimeout),
                m_indexer.startSpindexerCmd(),

                Commands.repeatingSequence(
                    Commands.none()
                )
            )
        )
        .finallyDo(
            () -> {
                deactivateOuttake();
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
        return Commands.parallel(
            m_shooter.shootFromCalc(),

            Commands.sequence(
                Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocityRotPerSec() >= ShooterK.kShooterSpunUpMinimumD))).withTimeout(ShooterK.kShooterSpunUpTimeout),
                m_indexer.startTunnelCmd(),
                Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocityRotPerSec() >= IndexerK.kTunnelSpunUpMinimumD)).withTimeout(IndexerK.kTunnelSpunUpTimeout),
                m_indexer.startSpindexerCmd()
            )
        )
        .finallyDo(
            () -> {
                deactivateOuttake();
            }
        );
    }

    /**
     * deactivates the outtake
     */
    public void deactivateOuttake() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
    }

    /**
     * @return the emergency barf command
     */
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

    public Command intakeShimmy() {
       return m_intake.shimmy();
    }

    /**
     * @param isShooting is if the robot is currently shooting
     * @return the unjam Command
     */
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
     * Deploys the intake to the given pos.
     * @param pos position to deploy to.
     * @return
     */
    public Command intakeTo(IntakeArmPosition pos) {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(pos)
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
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //deploy intake
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //run intake rollers
            m_intake.startIntakeRollers(),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //run spindexer
            m_intake.stopIntakeRollers(),
            m_indexer.setSpindexerVelocityCmd(IndexerK.kSpindexerShootRPS),
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
            m_shooter.setShooterVelocityCmd(RotationsPerSecond.zero()),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),

            //turret move
            m_shooter.m_turret.setTurretPosCmd(ShooterK.kTurretIntakeLockPos),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),
            m_shooter.m_turret.setTurretPosCmd(ShooterK.kHomePosition),

            //hood move
            m_shooter.m_hood.setHoodPosCmd(ShooterK.kHoodLockRots_double),
            Commands.waitSeconds(SuperstructureK.kLongOpsCheckPause),
            m_shooter.m_hood.setHoodPosCmd(0.1),

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
            Commands.print("================================= ACTIVATE INTAKE CMD ================================="),
            intake(() -> false).withTimeout(SuperstructureK.kShortOpsCheckIntakeTime),
            Commands.waitSeconds(SuperstructureK.kShortOpsCheckPause),

            //runs shooting cmd
            Commands.print("================================= ACTIVATE OUTTAKE CMD ================================="),
            activateOuttakeShotCalc().withTimeout(SuperstructureK.kShortOpsCheckShooterTime),
            Commands.waitSeconds(SuperstructureK.kShortOpsCheckPause),

            //Swerve test
            Commands.print("================================= START SWERVE OPSP CHECK ================================="),
            m_drivetrain.swerveAutomatedOpsCheck()
        );
    }
}
