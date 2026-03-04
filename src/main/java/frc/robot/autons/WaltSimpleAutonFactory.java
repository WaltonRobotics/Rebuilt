package frc.robot.autons;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterK;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.shooter.Shooter;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructre;
    private final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Shooter m_shooter;

    public WaltSimpleAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake, Shooter shooter) {
        m_superstructre = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
        m_shooter = shooter;
    }

    private Command preloadShot() {
        return Commands.sequence(
            // Commands.waitUntil(() -> (m_intake.m_isIntakeArmHomed && m_shooter.m_isTurretHomed)),
            Commands.waitSeconds(1),    //0.15
            m_superstructre.activateOuttake(ShooterK.kShooterRPS).withTimeout(2)
        );
    }

    private Command homingCmd() {
        return Commands.parallel(
            m_shooter.turretHomingCmd(false),
            m_intake.intakeArmCurrentSenseHoming()
        );
    }

    public Command rightOneSweep() {
        return Commands.sequence(
            Commands.sequence(
                homingCmd(),
                Commands.waitUntil(() -> m_shooter.m_isTurretHomed),  //TODO: should probably change this to check if is aiming at target (hub)
                preloadShot()
            ),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightSweep"),
                Commands.sequence(
                    Commands.waitSeconds(2),
                    m_superstructre.intake(false).withTimeout(6)
                )
            ),
            m_superstructre.activateOuttake(ShooterK.kShooterRPS).withTimeout(1)
        ).withName("rightOneSweep");
    }

    public Command rightTwoSweep() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry("RightSweep"),
                preloadShot()
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
                preloadShot()
            ),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightToDepot"),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    m_superstructre.activateIntake()
                )
            ),
            m_autoFactory.resetOdometry("RightDepotToShoot"),
            m_superstructre.deactivateIntake(IntakeArmPosition.SAFE),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightDepotToShoot"),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    m_superstructre.activateOuttake(ShooterK.kShooterRPS).withTimeout(2)
                )
            )
            
        );
    }

    public Command rightOutpostToShoot() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry("RightToOutpost"),
                preloadShot()
            ),
            m_autoFactory.resetOdometry("RightOutpostToNeutral"),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightOutpostToNeutral"),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    m_superstructre.activateOuttake(ShooterK.kShooterRPS).withTimeout(2),
                    Commands.waitSeconds(0.5),
                    m_intake.setIntakeArmPosCmd(IntakeArmPosition.SAFE),
                    Commands.waitSeconds(0.5),
                    m_superstructre.activateIntake()
                )
            )
        );
    }
}

