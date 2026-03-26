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

    /*TRUTH MACHINE SUPPLIERS*/
    // these aren't being used but i'm keebing them bc i spent too much time on them and they may be useful in the future
    private BooleanSupplier supp_intakeArmDeployed;
    private BooleanSupplier supp_intakeArmSafe;
    private BooleanSupplier supp_intakeArmShimmying;
    private BooleanSupplier supp_intaking;

    private BooleanSupplier supp_shooterReadyToShoot;
    private BooleanSupplier supp_tunnelReadyToShoot;

    private BooleanSupplier supp_turretIsLocked;
    private BooleanSupplier supp_isShooting;

    private BooleanSupplier supp_isBarfing;
    private BooleanSupplier supp_isUnjamming;

    private BooleanSupplier supp_canShoot;

    /* LOGGERS */

    
    /* CONSTRUCTOR */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;

        supp_intakeArmDeployed = () -> m_intake.intakeArmAtSpecifiedPos(IntakeArmPosition.DEPLOYED, 0.01);
        supp_intakeArmSafe = () -> m_intake.intakeArmAtSpecifiedPos(IntakeArmPosition.SAFE, 0.01);
        supp_intakeArmShimmying = () -> intakeArmShimmyCmd().isScheduled();
        supp_intaking = () -> (supp_intakeArmDeployed.getAsBoolean() && m_intake.rollersAtMaxSpeed(5));

        supp_shooterReadyToShoot = () -> (m_shooter.isShooterSpunUp() && (m_shooter.getShooterVelocity().gte(ShooterK.kShooterSpunUpMinimum)));
        supp_tunnelReadyToShoot = () -> (m_indexer.isTunnelSpunUp()) && (m_indexer.getTunnelVelocity().gte(IndexerK.kTunnelSpunUpMinimum));

        supp_turretIsLocked = () -> m_shooter.m_turret.getTurretLocked();
        supp_isShooting = () -> supp_shooterReadyToShoot.getAsBoolean() && supp_tunnelReadyToShoot.getAsBoolean();

        supp_isBarfing = () -> emergencyBarfCmd().isScheduled();
        supp_isUnjamming = () -> unjamCmd(supp_isShooting).isScheduled();

        supp_canShoot = () -> !supp_isBarfing.getAsBoolean();
    }

    /* SUBSYSTEM COMMANDS */

    //---INTAKE COMMANDS

    /**
     * @param isShooting if the robot is currently shooting
     * @return a Command that deploys the intake arm and runs the overall intaking logic
     */
    public Command intakeCmd(BooleanSupplier isShooting) {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.intakeArmAtTargetPos(0.01)).withTimeout(0.25),
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

    /**
     * @return a Command that turns off the intake rollers and puts the intake arm in the RETRACTED position
     */
    public Command retractIntakeCmd() {
        return m_intake.retractIntakeArmCmd();
    }

    /**
     * @return a Command that calls intakeArmShimmy (oscillates the intakeArm between the DEPLOYED and SHIMMY position)
     */
    public Command intakeArmShimmyCmd() {
       return m_intake.intakeArmShimmy();
    }

    //---SHOOTING COMMANDS

    /**
     * @param RPS Supplier for the desired shooter speed
     * @return a Command that starts and controls the overall shooting sequence logic
     */
    public Command shootCmd(Supplier<AngularVelocity> RPS) {
        if (supp_canShoot.getAsBoolean()) {
            return Commands.parallel(
                m_shooter.setShooterVelocityCmdSupp(RPS),

                Commands.sequence(
                    Commands.waitUntil(supp_shooterReadyToShoot).withTimeout(ShooterK.kShooterSpunUpTimeout),
                    m_indexer.startTunnelCmd(),
                    Commands.waitUntil(supp_tunnelReadyToShoot).withTimeout(IndexerK.kTunnelSpunUpTimeout),

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
        } else {
            return Commands.none();
        }
    }

    /**
     * @return a Command that starts and controls the overall shooting sequence logic using the shotClauclator value
     */
    public Command shootWithShotCalcCmd() {
        if (supp_canShoot.getAsBoolean()) {
            return Commands.parallel(
                m_shooter.shootFromCalc(),

                Commands.sequence(
                    Commands.waitUntil(supp_shooterReadyToShoot).withTimeout(ShooterK.kShooterSpunUpTimeout),
                    m_indexer.startTunnelCmd(),
                    Commands.waitUntil(supp_tunnelReadyToShoot).withTimeout(IndexerK.kTunnelSpunUpTimeout),

                    m_indexer.startSpindexerCmd()
                )
            )
            .finallyDo(
                () -> {
                    stopShooting();
                }
            );
        } else {
            return Commands.none();
        }
    }

    /**
     * Stops shooting by turning off the spindexer, tunnel, and shooter
     */
    public void stopShooting() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
    }

    //---COPE COMMANDS

    /**
     * @return a Command that ejects fuel from the intake and shooter while active
     */
    public Command emergencyBarfCmd() {
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

    public Command lockTurret() {
        return m_shooter.m_turret.setTurretLockCmd(true);
    }

    public Command unlockTurret() {
        return m_shooter.m_turret.setTurretLockCmd(false);
    }

    /**
     * @param isShooting if the shooter is currently shooting
     * @return a Command that unjams the shooter by running the spindexer, tunnel, and shooter (if it is running) backwards
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
}