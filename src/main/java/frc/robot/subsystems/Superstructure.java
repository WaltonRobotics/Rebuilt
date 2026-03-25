package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerK;
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

    /* LOGGERS */

    
    /* CONSTRUCTOR */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* SUBSYSTEM COMMANDS */

    //---INTAKE COMMANDS

    /**
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

                m_shooter.m_turret.setIntaking(!shooting);
                m_intake.setIntakeRollersVelocity(12);
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

    public Command retractIntakeCmd() {
        return m_intake.retractIntakeArmCmd();
    }

    public Command intakeArmShimmy() {
       return m_intake.intakeArmShimmy();
    }

    /**
     * Turns on spinner and exhaust and sets shooter speed to RPS.
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command shoot(Supplier<AngularVelocity> RPS) {
        return Commands.parallel(
            m_shooter.setShooterVelocityCmdSupp(RPS),

            Commands.sequence(
                Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocity().gte(ShooterK.kShooterSpunUpMinimum)))).withTimeout(ShooterK.kShooterSpunUpTimeout),
                m_indexer.startTunnelCmd(),
                Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocity().gte(IndexerK.kTunnelSpunUpMinimum))).withTimeout(IndexerK.kTunnelSpunUpTimeout),

                m_indexer.startSpindexerCmd(),

                Commands.repeatingSequence(
                    Commands.none()
                )
            )
        )
        .finallyDo(
            () -> {
                stopShooting();
            }
        );
    }

    /**
     * Turns on spinner and exhaust and sets shooter speed to CALCULATE SHOT RPS
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command shootWithShotCalc() {
        return Commands.parallel(
            m_shooter.shootFromCalc(),

            Commands.sequence(
                Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocity().gte(ShooterK.kShooterSpunUpMinimum)))).withTimeout(ShooterK.kShooterSpunUpTimeout),
                m_indexer.startTunnelCmd(),
                Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocity().gte(IndexerK.kTunnelSpunUpMinimum))).withTimeout(IndexerK.kTunnelSpunUpTimeout),

                m_indexer.startSpindexerCmd()
            )
        )
        .finallyDo(
            () -> {
                stopShooting();
            }
        );
    }

    /**
     * Turns off spinner, exhaust, and shooter.
     * <p>
     * Note: does not move turret or hood.
     */
    public void stopShooting() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);

    }

    public Command emergencyBarf() {
        return Commands.startEnd(
            () -> {
                m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPS);
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPS);
                m_shooter.setShooterVelocity(ShooterK.kShooterBarfRPS);
                m_intake.setIntakeRollersVelocity(-12);
            },
            () -> {
                m_intake.setIntakeRollersVelocity(0);
                m_intake.setIntakeArmPos(IntakeArmPosition.SAFE);
                m_shooter.setShooterVelocity(RotationsPerSecond.zero());
                m_indexer.setSpindexerVelocity(RotationsPerSecond.zero());
                m_indexer.setTunnelVelocity(RotationsPerSecond.zero());
            }
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
}