package frc.robot.autons;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterK;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructre;
    private final AutoFactory m_autoFactory;
    private final Intake m_intake;

    public WaltSimpleAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake) {
        m_superstructre = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
    }

    private Command preloadShot() {
        return Commands.sequence(
            Commands.waitSeconds(0.15),  //TODO: tune number
            m_superstructre.activateOuttake(ShooterK.kShooterMaxRPS)
        );
    }

    public Command rightOneSweep() {
        return Commands.parallel(
            m_autoFactory.trajectoryCmd("RightSweep"),
            Commands.sequence(
                // m_intake.setIntakeArmPosCmd(IntakeArmPosition.SAFE),
                Commands.waitSeconds(2),
                m_superstructre.activateIntake(),
                Commands.waitSeconds(3),
                m_superstructre.deactivateIntake(IntakeArmPosition.SAFE)
                // Commands.waitSeconds(1),
                // m_superstructre.activateOuttake(ShooterK.kShooterMaxRPS),
                // Commands.waitSeconds(0.15),
                // m_superstructre.deactivateOuttake()
            )
        ).withName("rightOneSweep");
    }

    public Command rightTwoSweep() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry("RightSweep")
                // preloadShot()
            ),
            // m_superstructre.deactivateOuttake(),
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
            // m_superstructre.deactivateOuttake(),
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
                    Commands.waitSeconds(1)
                    // m_superstructre.activateOuttake(ShooterK.kShooterMaxRPS)
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
            // m_superstructre.deactivateOuttake(),
            m_autoFactory.resetOdometry("RightOutpostToNeutral"),
            Commands.parallel(
                m_autoFactory.trajectoryCmd("RightOutpostToNeutral"),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    // m_superstructre.activateOuttake(ShooterK.kShooterMaxRPS),
                    Commands.waitSeconds(0.5),
                    // m_superstructre.deactivateOuttake(),
                    m_intake.setIntakeArmPosCmd(IntakeArmPosition.SAFE),
                    Commands.waitSeconds(0.5),
                    m_superstructre.activateIntake()
                )
            )
        );
    }
}

