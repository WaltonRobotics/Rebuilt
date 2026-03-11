
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
import frc.util.WaltLogger.StringLogger;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.SuperstructureK.*;
import static frc.robot.Constants.ShooterK;
import static frc.robot.Constants.IntakeK;

import java.util.function.BooleanSupplier;

public class Superstructure extends SubsystemBase {
    /* SUBSYSTEMS */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    /* LOGGERS */
    StringLogger log_intakeState = new StringLogger(kLogTab, "intakeState");
    StringLogger log_indexerState = new StringLogger(kLogTab, "indexerState");
    StringLogger log_shooterState = new StringLogger(kLogTab, "shooterState");

    IntakeState m_intakeState = IntakeState.RETRACTED;
    IndexerState m_IndexerState = IndexerState.IDLE;

    /* CONSTRUCTOR */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* BUTTON BIND SEQUENCES */
    //---NORMAL BINDS
    public Command intake(BooleanSupplier isPassing) {
        return Commands.sequence(
            intakeArmToPos(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.isIntakeArmAtPos()).withTimeout(0.25),
            Commands.runEnd(
            () -> {
                m_shooter.setShotCalc(false);
                m_shooter.setTurretPos(Rotations.of(-0.250));
                m_intake.setIntakeRollersVelocity(Constants.IntakeK.kIntakeRollersMaxRPS);
                m_indexer.setSpindexerVelocity(isPassing.getAsBoolean() ? Constants.IndexerK.kSpindexerShootRPS : Constants.IndexerK.kSpindexerIntakeRPS);
            }, 
            () -> {
                Commands.run(() -> m_shooter.setShotCalcCmd(true));
                m_shooter.setShotCalc(true);
                m_intake.setIntakeRollersVelocity(RotationsPerSecond.of(0));   //TODO: add a isNear0Vel for rollers so we don't bring to safe until rollers are low speed
                m_indexer.stopSpindexer();
                // m_intake.setIntakeArmPos(IntakeArmPosition.SAFE);
            })
        );
    }

    /**
     * Turns off rollers and spinner and moves deploy to RETRACTED.
     */
    public Command retractIntake() {
        return Commands.sequence(
            m_intake.stopIntakeRollers(),
            intakeArmToPos(IntakeArmPosition.RETRACTED),
            m_indexer.stopSpindexerCmd()
        );
    }

    /**
     * Turns on spinner and exhaust and sets shooter speed to RPS.
     * <p>
     * Note: does not move turret or hood.
     * @param RPS the speed for the shooter
     */
    public Command shoot(AngularVelocity RPS) {
        return Commands.parallel(
            startShootingSequence(RPS)
                .onlyWhile(() -> m_shooter.isShooterSpunUp())
                .andThen(Commands.waitUntil(() -> m_shooter.isShooterSpunUp()))
                .repeatedly()
            // m_intake.shimmy()
        ).finallyDo(
            () -> stopShooting()
        );
    }

    /**
     * Starts shooting by waiting until the shooter flywheel is spun up, then turning on the tunnel and spindexer.
     * @param RPS RPS to set shooter velocity to.
     */
    public Command startShootingSequence(AngularVelocity RPS) {
        return Commands.sequence(
            m_shooter.setShooterVelocityCmd(RPS),
            Commands.waitUntil(() -> m_shooter.isShooterSpunUp()).withTimeout(3),
            m_indexer.startTunnelCmd(),
            m_indexer.startSpindexerCmd()
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
        // m_shooter.setHoodPosition(Degrees.of(1));
    }

    /**
     * Ejects fuel out of the intake and shooter in a controlled yet rapid rate.
     */
    public Command emergencyBarf() {
        return Commands.startEnd(
            () -> {
                m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
                // m_shooter.setHoodPosition(ShooterK.kHoodMaxDegs);
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPS);
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPS);
                m_shooter.setShooterVelocity(ShooterK.kShooterBarfRPS);
                m_intake.setIntakeRollersVelocity(IntakeK.kIntakeRollersMaxRPS.times(-1));
            },
            () -> {
                m_intake.setIntakeRollersVelocity(RotationsPerSecond.of(0));
                m_intake.setIntakeArmPos(IntakeArmPosition.SAFE);
                m_shooter.setShooterVelocity(RotationsPerSecond.of(0));
                // m_shooter.setHoodPosition(ShooterK.kHoodSafeDegs);
                m_indexer.setSpindexerVelocity(RotationsPerSecond.of(0));
                m_indexer.setTunnelVelocity(RotationsPerSecond.of(0));
            }
        );
    }

    /**
     * Oscillates the intake back and forth to unstick balls stuck in the intake.
     */
    public Command shimmy() {
       return m_intake.shimmy();
    }

    // TODO: potentially delete following 4 commands as they currently are useless and probably will stay useless
    // /**
    //  * Initiates passing by activating intake and outtake.
    //  */
    // public Command startPassing() {
    //     return Commands.sequence(
    //         activateIntake(),
    //         activateOuttake(ShooterK.kShooterRPS)
    //     );
    // }

    /**
     * Exits passing mode by deactivating intake with deploy to SAFE and deactivating outtake.
     */
    // public Command stopPassing() {
    //     return Commands.sequence(
    //         deactivateIntake(IntakeArmPosition.SAFE),
    //         deactivateOuttake(),
    //     );
    // }

    // public Command intakeWhilePassing() {
    //     return Commands.runEnd(
    //         () -> {
    //             m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
    //             if (m_intake.isIntakeArmAtPos()) {
    //                 m_intake.setIntakeRollersVelocity(Constants.IntakeK.kIntakeRollersMaxRPS);
    //             }
    //         }, 
    //         () -> {
    //             m_intake.setIntakeRollersVelocity(RotationsPerSecond.of(0));   //TODO: add a isNear0Vel for rollers so we don't bring to safe until rollers are low speed
    //             // m_intake.setIntakeArmPos(IntakeArmPosition.SAFE);
    //         }
    //     );
    // }

    // /**
    //  * Turns on rollers and spinner moves deploy to deployed positon.
    //  */
    // public Command activateIntake() {
    //     return Commands.sequence(
    //         m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
    //         Commands.waitUntil(() -> m_intake.isIntakeArmAtPos()),
    //         m_intake.startIntakeRollers(),
    //         m_indexer.setSpindexerVelocityCmd(Constants.IndexerK.kSpindexerIntakeRPS)
    //     );
    // }

    //---OVERRIDE BINDS
    /**
     * Sets the shooter speed to max.
     */
    public Command maxShooter() {
        return Commands.sequence(
            m_shooter.setShooterVelocityCmd(ShooterK.kShooterRPS)
            // m_shooter.setShooterVelocityCmd(RotationsPerSecond.of(50)),
        );
    }

    /**
     * Stops the shooter.
     */
    public Command stopShooter() {
        return m_shooter.setShooterVelocityCmd(ShooterK.kShooterZeroRPS);
    }

    /**
     * Rotates the turret to the given degs
     * @param degs degrees to rotate to.
     */
    public Command turretTo(Angle degs) {
        return m_shooter.setTurretPosCmd(Rotations.of(degs.in(Rotations)));
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
     * Starts the indexer spindexer.
     */
    public Command startSpindexerCmd() {
        return m_indexer.startSpindexerCmd();
    }

    /**
     * Stops the indexer spindexer.
     */
    public Command stopSpindexerCmd() {
        return m_indexer.stopSpindexerCmd();
    }

    /**
     * Runs the shooter, tunnel, and spindexer in reverse direction to unjam any balls potentially stuck in the shooting system.
     */
    public Command unjamCmd() {
        return Commands.runEnd(
            () -> {
                m_indexer.setSpindexerVelocity(Constants.IndexerK.kSpindexerShootRPS.times(-1));
                m_indexer.setTunnelVelocity(Constants.IndexerK.kTunnelShootRPS.times(-1));
                m_shooter.setShooterVelocity(Constants.ShooterK.kShooterRPS.times(-1));
            }, () -> {
                m_indexer.setSpindexerVelocity(RotationsPerSecond.of(0));
                m_indexer.setTunnelVelocity(RotationsPerSecond.of(0));
                m_shooter.setShooterVelocity(RotationsPerSecond.of(0));
            }
        );
    }

    /**
     * Starts the indexer tunnel.
     */
    public Command startTunnelCmd() {
        return m_indexer.startTunnelCmd();
    }

    /**
     * Stops the indexer tunnel.
     */
    public Command stopTunnelCmd() {
        return m_indexer.stopTunnelCmd();
    }

    /**
     * Starts the intake rollers.
     */
    public Command startIntakeRollers() {
        return m_intake.startIntakeRollers();
    }

    /**
     * Stops the intake rollers.
     */
    public Command stopIntakeRollers() {
        return m_intake.stopIntakeRollers();
    }

    /**
     * Deploys the intake arm to the given pos.
     * @param pos position to deploy to.
     * @return
     */
    public Command intakeArmToPos(IntakeArmPosition pos) {
        return m_intake.setIntakeArmPosCmd(pos);
    }

    @Override
    public void periodic() {
        log_intakeState.accept(m_intakeState.name());
        log_indexerState.accept(m_IndexerState.name());
        // log_shooterState.accept();
    }

    /* STATE ENUMS */
    private enum IntakeState {
        RETRACTED,
        DEPLOYED,
        HOMING,
        HOMED,
        SHIMMYING
    }

    private enum IndexerState {
        INTAKING,
        SHOOTING,
        UNJAMMING,
        IDLE
    }

    // for later once shooter gets disassembled and reassembled
    private enum ShooterState {

    }
}
