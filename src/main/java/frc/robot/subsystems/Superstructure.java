package frc.robot.subsystems;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.system.Timer;

import frc.robot.Constants.IndexerK;
import frc.robot.Constants.IntakeK;
import frc.robot.Constants.ShooterK;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.shooter.Shooter;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.function.BooleanSupplier;

public class Superstructure implements Mechanism {
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
    public Command intake(BooleanSupplier isShooting) {
        // return Commands.sequence(
        //     m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
        //     Commands.waitUntil(() -> m_intake.isIntakeArmAtDest()).withTimeout(0.25),
        //     m_intake.setIntakeRollersVelocityCmd(isShimmying.getAsBoolean() ? kIntakeRollersShimmyVolts : kIntakeRollersIntakeVolts),
        //     Commands.run(
        //     () -> {
        //         boolean shooting = isShooting.getAsBoolean();
        //         // m_shooter.m_turret.setIntaking(!shooting);
        //         m_indexer.setSpindexerVelocity(shooting ? IndexerK.kSpindexerShootRPSD : IndexerK.kSpindexerIntakeRPSD); 
        //         m_indexer.setTunnelVelocity(shooting ? IndexerK.kTunnelShootRPSD : IndexerK.kTunnelIntakeRPSD);
        //     })
        //     ).finallyDo(
        //     () -> {
        //         boolean shooting = isShooting.getAsBoolean();
        //         boolean shimmying = isShimmying.getAsBoolean();
        //         // m_shooter.m_turret.setIntaking(false);
        //         if (!shimmying)
        //             m_intake.setIntakeRollersVelocity(0);
        //         if (!shooting) {
        //             m_indexer.setSpindexerVelocity(0);
        //             m_indexer.setTunnelVelocity(0);
        //         }
        //     }
        // );

        return m_intake.run(coroutine -> {
            m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
            coroutine.waitUntil(() -> m_intake.isIntakeArmAtDest(), Seconds.of(0.25));
            m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersIntakeVolts);

            if (!isShooting.getAsBoolean()) { // intakeIndex only when not shooting, as during shooting the shot occupies indexer
                coroutine.await(
                    m_indexer.runRepeatedly(() -> {
                        m_indexer.setSpindexerVelocity(IndexerK.kSpindexerIntakeRPSD); 
                        m_indexer.setTunnelVelocity(IndexerK.kTunnelIntakeRPSD);
                    }).named("intakeIndexing")
                );
            }

            coroutine.park();
        }).whenExited(() -> {
            boolean shooting = isShooting.getAsBoolean();
            m_intake.setIntakeRollersVelocity(0);
            if (!shooting) {
                 m_indexer.setSpindexerVelocity(0);
                  m_indexer.setTunnelVelocity(0);
            }
        }).named("intake");
    }

    public Command retractIntake() {
        // return Commands.sequence(
        //     m_intake.stopIntakeRollers(),
        //     m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        // );

        return m_intake.run(coroutine -> {
            coroutine.await(m_intake.stopIntakeRollers());
            coroutine.await(m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED));
        }).named("retractIntake");
    }

    // /**
    //  * Turns on spinner and exhaust and sets shooter speed to RPS.
    //  * <p>
    //  * Note: does not move turret or hood.
    //  * @param RPS the speed for the shooter
    //  */
    // public Command activateShooter(Supplier<AngularVelocity> RPS) {
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
    //             deactivateShooter();
    //         }
    //     );
    // }

    /**
     * Turns on spinner and exhaust and sets shooter speed to CALCULATE SHOT RPS
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command activateShooterShotCalc() {
        // return Commands.parallel(
        //     m_shooter.shootFromCalc(),
        //     m_shooter.hoodFromCalc(),

        //     Commands.sequence(
        //         Commands.waitUntil(() -> m_shooter.m_hood.atPosition()).withTimeout(ShooterK.kHoodAtPosTimeout),
        //         Commands.waitUntil(() -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocityRotPerSec() >= ShooterK.kShooterSpunUpMinimumD))).withTimeout(ShooterK.kShooterSpunUpTimeout),
        //         Commands.parallel(
        //             Commands.run(() -> m_indexer.setTunnelVelocity(kTunnelShootRPSD)),
        //             Commands.sequence(
        //                 Commands.waitUntil(() -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocityRotPerSec() >= IndexerK.kTunnelSpunUpMinimumD)).withTimeout(IndexerK.kTunnelSpunUpTimeout),
        //                 Commands.run(() -> m_indexer.setSpindexerVelocity(kSpindexerShootRPSD))
        //             )
        //         )
        //     )
        // )
        // .finallyDo(() -> {
        //     deactivateShooter();
        //     hoodBackToSafe();
        // });

        return Command.requiring(m_shooter, m_indexer).executing(coroutine -> {
            coroutine.fork(
                m_shooter.shootFromCalc(),
                m_shooter.hoodFromCalc()
            );
            coroutine.waitUntil(() -> m_shooter.m_hood.atPosition(), Seconds.of(ShooterK.kHoodAtPosTimeout));
            coroutine.waitUntil(() -> (
                m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocityRotPerSec() >= ShooterK.kShooterSpunUpMinimumD)),
                ShooterK.kShooterSpunUpTimeout
            );
            coroutine.await(m_indexer.setTunnelVelocityCmd(RotationsPerSecond.of(IndexerK.kTunnelShootRPSD)));
            coroutine.waitUntil(() -> 
                (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocityRotPerSec() >= IndexerK.kTunnelSpunUpMinimumD),
                IndexerK.kTunnelSpunUpTimeout
            );
            coroutine.await(m_indexer.setSpindexerVelocityCmd(RotationsPerSecond.of(IndexerK.kSpindexerShootRPSD)));
            coroutine.park();
        }).whenExited(() -> {
            deactivateShooter();
            hoodBackToSafe();
        }).named("activateShooterShotCalc");
    }

    public Command activateHoodShotCalc() {
        // return m_shooter.hoodFromCalc()
        //     .finallyDo(() -> {
        //         hoodBackToSafe();
        // });

        return m_shooter.run(coroutine -> {
            coroutine.await(m_shooter.hoodFromCalc());
        }).whenExited(() -> {
            hoodBackToSafe();
        }).named("activateHoodShotCalc");
    }

    public Command spinUpFlywheel() {
        // return m_shooter.shootFromCalc()
        // .finallyDo(() -> {
        //     turnOffFlywheel();
        // });

        return m_shooter.run(coroutine -> {
            coroutine.await(m_shooter.shootFromCalc());
        }).whenExited(() -> {
            turnOffFlywheel();
        }).named("spinUpFlywheel");
    }

    public Command hoodAndFlywheelShotCalc() {
        // return Commands.parallel(
        //     m_shooter.hoodFromCalc(),
        //     m_shooter.shootFromCalc()
        // )
        // .finallyDo(() -> {
        //     hoodBackToSafe();
        //     turnOffFlywheel();
        // });

        return m_shooter.run(coroutine -> {
            coroutine.awaitAll(
                m_shooter.hoodFromCalc(),
                m_shooter.shootFromCalc()
            );
        }).whenExited(() -> {
            hoodBackToSafe();
            turnOffFlywheel();
        }).named("hoodAndFlywheelShotCalc");
    }

    /**
     * deactivates the shooter
     */
    public void deactivateShooter() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
    }

    public void turnOffFlywheel() {
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
    }

    public void hoodBackToSafe() {
        m_shooter.m_hood.setHoodPos(ShooterK.kHoodEmergencyRotsD);
    }

    /**
     * @return the emergency barf command
     */
    public Command emergencyBarf() {
        // return Commands.startEnd(
        //     () -> {
        //         m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
        //         m_shooter.m_turret.lockAndSetTurretLockPos(ShooterK.kTurretBarfPos.magnitude());
        //         m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD);
        //         m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD);
        //         m_shooter.setShooterVelocity(ShooterK.kShooterBarfRPS);
        //         m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersBarfVolts);
        //     },
        //     () -> {
        //         m_intake.setIntakeRollersVelocity(0);
        //         m_shooter.m_turret.setTurretLock(false);
        //         m_shooter.setShooterVelocity(RotationsPerSecond.zero());
        //         m_indexer.setSpindexerVelocity(0);
        //         m_indexer.setTunnelVelocity(0);
        //     }
        // );

        return Command.requiring(m_shooter, m_indexer, m_intake).executing(coroutine -> {
            m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
            m_shooter.m_turret.lockAndSetTurretLockPos(ShooterK.kTurretBarfPos.magnitude());
            m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD);
            m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD);
            m_shooter.setShooterVelocity(ShooterK.kShooterBarfRPS);
            m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersBarfVolts);
            coroutine.park();
        }).whenExited(() -> {
            m_intake.setIntakeRollersVelocity(0);
            m_shooter.m_turret.setTurretLock(false);
            m_shooter.setShooterVelocity(RotationsPerSecond.zero());
            m_indexer.setSpindexerVelocity(0);
            m_indexer.setTunnelVelocity(0);
        }).named("emergencyBarf");
    }


    /**
     * @return the emergency barf command
     */
    public Command emergencyBarfOnlyIntake() {
        // return Commands.startEnd(
        //     () -> {
        //         m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
        //         m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersBarfVolts);
        //     },
        //     () -> {
        //         m_intake.setIntakeRollersVelocity(0);
        //     }
        // );

        return m_intake.run(coroutine -> {
            m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
            m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersBarfVolts);
            coroutine.park();
        }).whenExited(() -> {
            m_intake.setIntakeRollersVelocity(0);
        }).named("emergencyBarfOnlyIntake");
    }

    public Command intakeShimmy() {
        // return Commands.repeatingSequence(
        //     intake(isShooting, () -> true).withTimeout(0.2),
        //     m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED),
        //     Commands.waitSeconds(0.2)
        // ).finallyDo(
        //     () -> m_intake.setIntakeRollersVelocity(0)
        // );

        Timer intakeShimmyTimer = new Timer();

        return m_intake.runRepeatedly(() -> {
            if (!intakeShimmyTimer.isRunning()) {
                intakeShimmyTimer.restart();
            }
            boolean isDeployed = (intakeShimmyTimer.get()) % 0.4 < 0.2;
            m_intake.setIntakeArmPos(isDeployed ? IntakeArmPosition.DEPLOYED : IntakeArmPosition.RETRACTED);
            m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersShimmyVolts);

        }).whenExited(() -> {
            m_intake.setIntakeRollersVelocity(0);
            intakeShimmyTimer.stop();
        }).named("intakeShimmy");
    }

    /**
     * @param isShooting is if the robot is currently shooting
     * @return the unjam Command
     */
    public Command unjamCmd(BooleanSupplier isShooting) {
        // return Commands.runEnd(
        //     () -> {
        //         m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD * -1);
        //         m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD * -1);
        //         if (!isShooting.getAsBoolean()) {
        //             m_shooter.setShooterVelocity(ShooterK.kShooterRPS.unaryMinus());
        //         }
        //     }, () -> {
        //         if (!isShooting.getAsBoolean()) {
        //             m_shooter.setShooterVelocity(RotationsPerSecond.zero());
        //             m_indexer.setSpindexerVelocity(0);
        //             m_indexer.setTunnelVelocity(0);
        //         }
        //         else if (isShooting.getAsBoolean()) {
        //             m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD);
        //             m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD);
        //         }
        //     }
        // );

        return Command.noRequirements(coroutine -> {
            m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD * -1);
            m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD * -1);
            if (!isShooting.getAsBoolean()) {
                m_shooter.setShooterVelocity(ShooterK.kShooterRPS.unaryMinus());
            }
            coroutine.park();
        }).whenExited(() -> {
            if (!isShooting.getAsBoolean()) {
                m_shooter.setShooterVelocity(RotationsPerSecond.zero());
                m_indexer.setSpindexerVelocity(0);
                m_indexer.setTunnelVelocity(0);
            } else {
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPSD);
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPSD);
            }
        }).named("unjamCmd");
    }

    /**
     * Deploys the intake to the given pos.
     * @param pos position to deploy to.
     * @return
     */
    public Command intakeTo(IntakeArmPosition pos) {
        return m_intake.setIntakeArmPosCmd(pos);
    }
}
