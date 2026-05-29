
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerK;
import frc.robot.Constants.IntakeK;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.shooter.Shooter;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK;
import static frc.robot.Constants.IntakeK.kIntakeRollersIntakeVolts;
import static frc.robot.Constants.IntakeK.kIntakeRollersShimmyVolts;

import java.util.function.BooleanSupplier;

public class Superstructure extends SubsystemBase {
    // --- SUBSYSTEM REFERENCES ---
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;
    
    // ======================================================================
    // CONSTRUCTOR
    // ======================================================================
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    // ======================================================================
    // SEQUENCES
    // ======================================================================

    // --- INTAKE ---
    /**
     * @param isShooting is if the robot is shooting
     * @return the intake Command
     */
    public Command intake(BooleanSupplier isShooting, BooleanSupplier isShimmying) {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.isIntakeArmAtDest()).withTimeout(0.25),
            m_intake.setIntakeRollersVelocityCmd(isShimmying.getAsBoolean() ? kIntakeRollersShimmyVolts : kIntakeRollersIntakeVolts),
            Commands.run(
            () -> {
                boolean shooting = isShooting.getAsBoolean();
                // m_shooter.m_turret.setIntaking(!shooting);
                m_indexer.setSpindexerVelocity(shooting ? IndexerK.kSpindexerShootRPSD : IndexerK.kSpindexerIntakeRPSD);
            })
        ).finallyDo(
            () -> {
                boolean shooting = isShooting.getAsBoolean();
                boolean shimmying = isShimmying.getAsBoolean();
                // m_shooter.m_turret.setIntaking(false);
                if (!shimmying)
                    m_intake.setIntakeRollersVelocity(0);
                if (!shooting) {
                    m_indexer.setSpindexerVelocity(0);
                }
            }
        );
    }

    public Command retractIntake() {
        return Commands.sequence(
            m_intake.stopIntakeRollers(),
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        );
    }

    // ~~~~~~~~~~~~~~~~~
    // OUTTAKE
    // ~~~~~~~~~~~~~~~~~
    // /**
    //  * Turns on spinner and exhaust and sets shooter speed to RPS.
    //  * <p>
    //  * Note: does not move turret or hood.
    //  * @param RPS the speed for the shooter
    //  */
    // public Command activateOuttake(Supplier<AngularVelocity> RPS) {
    //     return Commands.parallel(
    //         m_shooter.setShooterVelocityCmdSupp(RPS),

    //         Commands.sequence(
    //             Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocityRotPerSec() >= ShooterK.kShooterSpunUpMinimumD))).withTimeout(ShooterK.kShooterSpunUpTimeout),
    //             Commands.runOnce(() -> m_indexer.setTunnelVelocity(Indexer.tunnelRPSFromShooter(RPS.get().in(RotationsPerSecond)))),
    //             Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocityRotPerSec() >= IndexerK.kTunnelSpunUpMinimumD)).withTimeout(IndexerK.kTunnelSpunUpTimeout),
    //             Commands.runOnce(() -> m_indexer.setSpindexerVelocity(Indexer.spindexerRPSFromShooter(RPS.get().in(RotationsPerSecond)))),

    //             Commands.repeatingSequence(
    //                 Commands.none()
    //             )
    //         )
    //     )
    //     .finallyDo(
    //         () -> {
    //             deactivateOuttake();
    //         }
    //     );
    // }

    /**
     * Turns on spinner and exhaust and sets shooter speed to CALCULATE SHOT RPS
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command activateOuttakeShotCalc() {
        return Commands.parallel(
            m_shooter.shootFromCalc(),
            m_shooter.hoodFromCalc(),

            Commands.sequence(
                Commands.waitUntil(() -> m_shooter.m_hood.atPosition()).withTimeout(ShooterK.kHoodAtPosTimeout),
                Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocityRotPerSec() >= ShooterK.kShooterSpunUpMinimumD))).withTimeout(ShooterK.kShooterSpunUpTimeout),
                Commands.parallel(
                    Commands.run(() -> m_indexer.setTunnelVelocity(Indexer.tunnelRPSFromShooter(m_shooter.getShooterDesiredRotPerSecSupp()).getAsDouble())),
                    Commands.sequence(
                        Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocityRotPerSec() >= IndexerK.kTunnelSpunUpMinimumD)).withTimeout(IndexerK.kTunnelSpunUpTimeout),
                        Commands.run(() -> m_indexer.setSpindexerVelocity(Indexer.spindexerRPSFromShooter(m_shooter.getShooterDesiredRotPerSecSupp()).getAsDouble()))
                    )
                )
            )
        )
        .finallyDo(() -> {
            deactivateOuttake();
            hoodBackToSafe();
        });
    }

    // --- HOOD ---
    public Command activateHoodShotCalc() {
        return m_shooter.hoodFromCalc()
        .finallyDo(() -> {
            hoodBackToSafe();
        });
    }

    // --- FLYWHEEL ---
    public Command spinUpFlywheel() {
        return m_shooter.shootFromCalc()
        .finallyDo(() -> {
            turnOffFlywheel();
        });
    }

    // --- HOOD && FLYWHEEL ---
    public Command hoodAndFlywheelShotCalc() {
        return Commands.parallel(
            m_shooter.hoodFromCalc(),
            m_shooter.shootFromCalc()
        )
        .finallyDo( () -> {
            hoodBackToSafe();
            turnOffFlywheel();
        });
    }

    /**
     * Stops the tunnel, spindexer, and the shooter.
     */
    public void deactivateOuttake() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
    }

    // --- FLYWHEEL ---
    /**
     * Sets the Flywheel to a speed of 0 which will CoastOut
     */
    public void turnOffFlywheel() {
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
    }

    // --- HOOD ---
    /**
     * Sets the Hood back to a position that is able to go underneath the trench
     */
    public void hoodBackToSafe() {
        m_shooter.m_hood.setHoodPos(ShooterK.kHoodEmergencyRotsD);
    }

    /**
     * Command to get all balls out of the hopper, one way or another.
     * This reverses the intake rollers, and shoots out all the balls at a set LOCKED turret pose.
     * @return Command above
     */
    public Command emergencyBarf() {
        return Commands.startEnd(
            () -> {
                m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
                m_shooter.m_turret.lockAndSetTurretLockPos(ShooterK.kTurretBarfPos.magnitude());
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD);
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD);
                m_shooter.setShooterVelocity(ShooterK.kShooterBarfRPS);
                m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersBarfVolts);
            },
            () -> {
                m_intake.setIntakeRollersVelocity(0);
                m_shooter.m_turret.setTurretLock(false);
                m_shooter.setShooterVelocity(RotationsPerSecond.zero());
                m_indexer.setSpindexerVelocity(0);
                m_indexer.setTunnelVelocity(0);
            }
        );
    }


    /**
     * @return Command that reverses only the intake rollers
     */
    public Command emergencyBarfOnlyIntake() {
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

    /**
     * Shimmies the intake up and down while running the intake rollers at a slower speed
     * @param isShooting is the robot in a shooting state
     * @return the shimmy command
     */
    public Command intakeShimmy(BooleanSupplier isShooting) {
       return Commands.repeatingSequence(
            intake(isShooting, () -> true).withTimeout(0.5),
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED),
            Commands.waitSeconds(0.5)
       )
       .finallyDo(() -> {
            m_intake.stopIntakeRollers();
       });
    }

    /**
     * Runs the indexer in reverse, and if the robot is shooting, then the shooter flywheel WONT reverse, 
     * otherwise flywheel will reverse
     * @param isShooting is if the robot is currently shooting
     * @return the unjam Command
     */
    public Command unjamCmd(BooleanSupplier isShooting) {
        return Commands.runEnd(
            () -> {
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD * -1);
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD * -1);
                if (!isShooting.getAsBoolean()) {
                    m_shooter.setShooterVelocity(ShooterK.kShooterRPS.unaryMinus());
                }
            }, () -> {
                if (!isShooting.getAsBoolean()) {
                    m_shooter.setShooterVelocity(RotationsPerSecond.zero());
                    m_indexer.setSpindexerVelocity(0);
                    m_indexer.setTunnelVelocity(0);
                }
                else if (isShooting.getAsBoolean()) {
                    m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD);
                    m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD);
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
