
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
        return Commands.sequence(
            m_intake.stopIntakeRollers(),
            m_intake.setIntakeArmPosCmd(pos),
            m_indexer.stopSpindexerCmd()
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
            m_indexer.setSpindexerVelocityCmd(Constants.IndexerK.kSpindexerIntakeRPS)
        );
    }

    public Command intake(BooleanSupplier isPassing) {
        return Commands.sequence(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.DEPLOYED),
            Commands.waitUntil(() -> m_intake.isIntakeArmAtPos()).withTimeout(0.25),
            Commands.runEnd(
            () -> {
                m_shooter.setShotCalc(false);
                m_shooter.setTurretPos(Rotations.of(-0.250));
                m_intake.setIntakeRollersVelocity(Constants.IntakeK.kIntakeRollersMaxRPS);
                m_indexer.setSpindexerVelocity(isPassing.getAsBoolean() ? Constants.IndexerK.kSpindexerShootRPS : Constants.IndexerK.kSpindexerIntakeRPS);
            }, 
            () -> {
                if (m_intake.getIntakeArmStatorCurrent() < 40) {
                    Commands.run(() -> m_shooter.setShotCalcCmd(true));
                    m_shooter.setShotCalc(true);
                    m_intake.setIntakeRollersVelocity(RotationsPerSecond.of(0));   //TODO: add a isNear0Vel for rollers so we don't bring to safe until rollers are low speed
                    m_indexer.stopSpindexer();
                    // m_intake.setIntakeArmPos(IntakeArmPosition.SAFE);
                }
            })
        );
    }

    public Command intakeWhilePassing() {
        return Commands.runEnd(
            () -> {
                m_intake.setIntakeArmPos(IntakeArmPosition.DEPLOYED);
                if (m_intake.isIntakeArmAtPos()) {
                    m_intake.setIntakeRollersVelocity(Constants.IntakeK.kIntakeRollersMaxRPS);
                }
            }, 
            () -> {
                if (m_intake.getIntakeArmStatorCurrent() < 40) {
                    m_intake.setIntakeRollersVelocity(RotationsPerSecond.of(0));   //TODO: add a isNear0Vel for rollers so we don't bring to safe until rollers are low speed
                    // m_intake.setIntakeArmPos(IntakeArmPosition.SAFE);
                }
            }
        );
    }

    public Command startShootSequence(AngularVelocity RPS) {
        return Commands.sequence(
            m_shooter.setShooterVelocityCmd(RPS),
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
        return Commands.parallel(
            startShootSequence(RPS)
                .onlyWhile(() -> m_shooter.isShooterSpunUp())
                .andThen(Commands.waitUntil(() -> m_shooter.isShooterSpunUp()))
                .repeatedly()
            // m_intake.shimmy()
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
        // m_shooter.setHoodPosition(Degrees.of(1));
    }

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

    public Command shimmy() {
       return m_intake.shimmy();
    }

    /**
     * Initiates passing by activating intake and outtake.
     */
    public Command startPassing() {
        return Commands.sequence(
            activateIntake(),
            activateOuttake(ShooterK.kShooterRPS)
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
     * Starts the indexer spinner.
     */
    public Command startSpindexerCmd() {
        return m_indexer.startSpindexerCmd();
    }

    /**
     * Stops the indexer spinner.
     */
    public Command stopSpindexerCmd() {
        return m_indexer.stopSpindexerCmd();
    }

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
     * Starts the indexer exhaust.
     */
    public Command startTunnelCmd() {
        return m_indexer.startTunnelCmd();
    }

    /**
     * Stops the indexer exhaust.
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
     * Deploys the intake to the given pos.
     * @param pos position to deploy to.
     * @return
     */
    public Command intakeTo(IntakeArmPosition pos) {
        return m_intake.setIntakeArmPosCmd(pos);
    }
}
