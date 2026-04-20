
package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerK;
import frc.robot.Constants.IntakeK;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.shooter.Shooter;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK;
import static frc.robot.Constants.IntakeK.kIntakeRollersShimmyVolts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    /* SUBSYSTEMS */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;
    
    /* CONSTRUCTOR */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* BUTTON BIND SEQUENCES */
    /**
     * @param isShooting is if the robot is shooting
     * @return the intake Command
     */
    public Command intake(BooleanSupplier isShooting, double volts, boolean stopRollers) {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.isIntakeArmAtDest()).withTimeout(0.25),
            m_intake.setIntakeRollersVelocityCmd(volts),
            Commands.run(
            () -> {
                // boolean shooting = isShooting.getAsBoolean();
                // m_shooter.m_turret.setIntaking(!shooting);
                // m_indexer.setSpindexerVelocity(shooting ? IndexerK.kSpindexerShootRPS : IndexerK.kSpindexerIntakeRPS);
            })
        ).finallyDo(
            () -> {
                // boolean shooting = isShooting.getAsBoolean();
                // m_shooter.m_turret.setIntaking(false);
                if (stopRollers)
                    m_intake.setIntakeRollersVelocity(0) ;
                // if (!shooting) {
                //     m_indexer.setSpindexerVelocity(RotationsPerSecond.zero());
                // }
            }
        );
    }

    public Command retractIntake() {
        return Commands.sequence(
            m_intake.stopIntakeRollers(),
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
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
        .finallyDo(() -> {
            deactivateOuttake();
        });
    }

    public Command spinUpFlywheel() {
        return m_shooter.shootFromCalc()
        .finallyDo(() -> {
            turnOffFlywheel();
        });
    }

    /**
     * deactivates the outtake
     */
    public void deactivateOuttake() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
    }

    public void turnOffFlywheel() {
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
    }

    /**
     * @return the emergency barf command
     */
    public Command emergencyBarf() {
        return Commands.startEnd(
            () -> {
                m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
                m_shooter.m_turret.lockAndSetTurretLockPos(ShooterK.kTurretBarfPos.magnitude());
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPS);
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPS);
                m_shooter.setShooterVelocity(ShooterK.kShooterBarfRPS);
                m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersBarfVolts);
            },
            () -> {
                m_intake.setIntakeRollersVelocity(0);
                m_shooter.m_turret.setTurretLock(false);
                m_shooter.setShooterVelocity(RotationsPerSecond.zero());
                m_indexer.setSpindexerVelocity(RotationsPerSecond.zero());
                m_indexer.setTunnelVelocity(RotationsPerSecond.zero());
            }
        );
    }


    /**
     * @return the emergency barf command
     */
    public Command emergencyBarfNOSHOOT() {
        return Commands.startEnd(
            () -> {
                m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
                m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersBarfVolts);
            },
            () -> {
                m_intake.setIntakeRollersVelocity(0);
            }
        );
    }

    public Command intakeShimmy() {
       return Commands.repeatingSequence(
            intake(() -> false, kIntakeRollersShimmyVolts, false).withTimeout(0.5),
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED),
            Commands.waitSeconds(0.5)
       );
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
}
