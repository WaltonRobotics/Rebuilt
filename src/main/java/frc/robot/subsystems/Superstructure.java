
package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerK;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.shooter.Shooter;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterK;
import static frc.robot.Constants.IntakeK;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    /* SUBSYSTEMS */
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    /* LOGIC BOOLEANS */
    private BooleanSupplier supp_canShoot = () -> (false);
    private BooleanSupplier supp_canUnjam = () -> (false);
    private BooleanSupplier supp_canIntake = () -> (false);
    private BooleanSupplier supp_canShimmy = () -> (false);
    private BooleanSupplier supp_canRetractIntakeArm = () -> (false);

    private BooleanSupplier supp_isTurretDriverLocked = () -> (false);
    
    /* CONSTRUCTOR */
    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
    }

    /* INTAKING COMMANDS */

    /**
     * @return a Command that calls intakeArmShimmy (moves the intakeArm from DEPLOYED TO RETRACTED)
     */
    public Command intakeArmShimmy() {
       return m_intake.intakeArmShimmy().onlyIf(supp_canShimmy);
    }

    /**
     * @return a Command that retracts the intake arm
     */
    public Command retractIntakeArm() {
        return m_intake.retractIntakeArm().onlyIf(supp_canRetractIntakeArm);
    }

    /**
     * @return a Command that directly tells the arm to move; does not check boolean flags
     */
    public Command overrideIntakeArm(IntakeArmPosition pos) {
        return m_intake.setIntakeArmPosCmd(pos);
    }

    /**
     * @param isShooting is if the robot is currently shooting
     * @return the overall Command that runs the intaking logic
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

                    m_shooter.setIntaking(false);
                    m_intake.setIntakeRollersVelocity(RotationsPerSecond.of(0));
                    if (!shooting) {
                        m_indexer.setSpindexerVelocityCmd(RotationsPerSecond.of(0));
                    }
                }
            ).onlyIf(supp_canIntake)
        );
    }

    /* SHOOTING COMMANDS */

    /**
     * @param RPS is the desired shooter speed
     * @return a Command that starts the overall shooting sequence
     */
    public Command shoot(Supplier<AngularVelocity> RPS) {
        return Commands.parallel(
            m_shooter.setShooterVelocityCmdSupp(RPS),
            Commands.sequence(
                Commands.waitUntil(() -> m_shooter.isShooterSpunUp()).withTimeout(ShooterK.kShooterTimeout),
                m_indexer.startTunnelCmd(),
                m_indexer.startSpindexerCmd()
            )
        )
        .onlyIf(supp_canShoot)
        .finallyDo(() -> stopShooting());
    }

    /**
     * @return a Command that starts the overall shooting sequence; uses shotcalculator value directly
     */
    public Command shootWithShotCalc() {
        return Commands.parallel(
            m_shooter.shootFromCalc(),
            Commands.sequence(
                Commands.waitUntil(() -> m_shooter.isShooterSpunUp()).withTimeout(ShooterK.kShooterTimeout),
                m_indexer.startTunnelCmd(),
                m_indexer.startSpindexerCmd()
            )
        )
        .onlyIf(supp_canShoot)
        .finallyDo(() -> stopShooting());
    }

    /**
     * TO BE USED ONLY WITH THE SHOOTER RPS TESTINGWIDGET
     * @return a Command that just starts up the spindexer and tunnel
     */
    public Command shoot_TestingWidget() {
        return Commands.sequence(
            m_indexer.startTunnelCmd(),
            m_indexer.startSpindexerCmd()
        )
        .onlyIf(supp_canShoot)
        .finallyDo(() -> stopShooting_TestingWidget());
    }

    //---DEACTIVATE SHOOTING METHODS
    private void stopShooting() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        m_shooter.setShooterVelocity(ShooterK.kShooterZeroRPS);
        // m_shooter.setHoodPosition(Degrees.of(1));
    }

    private void stopShooting_TestingWidget() {
        m_indexer.stopSpindexer();
        m_indexer.stopTunnel();
        // m_shooter.setHoodPosition(Degrees.of(1));
    }

    //---OVERRIDE SHOOTING COMMANDS
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
     * @param isShooting is if the shooter is currently shooting
     * @return the unjamming Command sequence
     */
    public Command unjamCmd(BooleanSupplier isShooting) {
        return Commands.runEnd(
            () -> {
                m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPS.times(-1));
                m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPS.times(-1));
                if (!isShooting.getAsBoolean()) {
                    m_shooter.setShooterVelocity(ShooterK.kShooterRPS.times(-1));
                }
            }, () -> {
                if (!isShooting.getAsBoolean()) {
                    m_shooter.setShooterVelocity(RotationsPerSecond.of(0));
                    m_indexer.setSpindexerVelocity(RotationsPerSecond.of(0));
                    m_indexer.setTunnelVelocity(RotationsPerSecond.of(0));
                }
                else if (isShooting.getAsBoolean()) {
                    m_indexer.setSpindexerVelocity(IndexerK.kSpindexerShootRPS);
                    m_indexer.setTunnelVelocity(IndexerK.kTunnelShootRPS);
                }
            }
        ).onlyIf(supp_canUnjam);
    }

}