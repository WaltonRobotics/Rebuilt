package frc.robot.autons;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.Constants.ShooterK;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructure;
    private final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Shooter m_shooter;

    public WaltSimpleAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake, Shooter shooter) {
        m_superstructure = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
        m_shooter = shooter;
    }

    private Command homingCmd() {
        return Commands.parallel(
            m_shooter.turretHomingCmd(false),
            m_intake.intakeArmCurrentSenseHoming()
        );
    }

    private Command shootWithTimeout(double seconds) {
        return m_superstructure.activateOuttake(ShooterK.kShooterAutonRPS).withTimeout(seconds);
    }

    private Command oneSweep(boolean isLeft) {
        String trajName = isLeft ? "LeftSweep" : "RightSweep";

        return Commands.sequence(
            Commands.sequence(
                homingCmd(),
                Commands.waitUntil(() -> m_shooter.m_isTurretHomed),  //TODO: should probably change this to check if is aiming at target (hub)
                shootWithTimeout(2)
            ),
            Commands.parallel(
                m_autoFactory.trajectoryCmd(trajName),
                Commands.sequence(
                    Commands.waitSeconds(2.5),
                    m_superstructure.intake(() -> false).withTimeout(7.5)
                )
            ).withTimeout(AutonK.kOneSweepMaxTime),  //should i remove this and j make the last pose in the choreo path 
            shootWithTimeout(12)
        ).withName(trajName);
    }

    public Command rightOneSweep() {
        return oneSweep(false);
    }

    public Command leftOneSweep() {
        return oneSweep(true);
    }

    public Command rightTwoSweep() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry("RightSweep"),
                m_superstructure.activateOuttake(ShooterK.kShooterAutonRPS).withTimeout(2)
            ),
            rightOneSweep(),
            m_autoFactory.resetOdometry("RightTurnBack"),
            m_autoFactory.trajectoryCmd("RightTurnBack"),
            rightOneSweep()
        );
    }

    public Command rightDepotToShoot() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry("RightToDepot"),
                shootWithTimeout(2)
            ),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightToDepot"),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    m_superstructure.intake(() -> false).withTimeout(2)
                )
            ),
            m_autoFactory.resetOdometry("RightDepotToShoot"),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightDepotToShoot"),
                Commands.sequence(
                    m_superstructure.activateOuttake(ShooterK.kShooterRPS).withTimeout(2)
                )
            )
            
        );
    }

    public Command rightOutpostToShoot() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry("RightToOutpost"),
                shootWithTimeout(2)
            ),
            m_autoFactory.resetOdometry("RightOutpostToNeutral"),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightOutpostToNeutral"),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    m_superstructure.activateOuttake(ShooterK.kShooterRPS).withTimeout(2),
                    Commands.waitSeconds(0.5),
                    m_intake.setIntakeArmPosCmd(IntakeArmPosition.SAFE),
                    Commands.waitSeconds(0.5),
                    m_superstructure.intake(() -> false).withTimeout(2)
                )
            )
        );
    }
}

