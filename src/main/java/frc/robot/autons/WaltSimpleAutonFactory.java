package frc.robot.autons;

import static frc.robot.Constants.ShooterK.kShooterAutonCloseRPS;
import static frc.robot.Constants.ShooterK.kShooterAuton_EndSweep_RPS;

import choreo.auto.AutoFactory;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.Constants.ShooterK;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructure;
    private final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Swerve m_swerve;

    public WaltSimpleAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake, Shooter shooter, Swerve swerve) {
        m_superstructure = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
        m_shooter = shooter;
        m_swerve = swerve;
    }

    private Command homingCmd() {
        return Commands.parallel(
            m_shooter.turretHomingCmd(),
            m_intake.intakeArmCurrentSenseHoming()
        );
    }

    private Command shootWithTimeout(AngularVelocity speed, double seconds) {
        return m_superstructure.activateOuttake(speed).withTimeout(seconds);
    }

    private Command runTraj(String name, double timeoutSecs) {
        return m_autoFactory.trajectoryCmd(name).withTimeout(timeoutSecs).andThen(m_swerve.xBrake());
    }

    private Command preload_oneSweep(boolean isLeft) {
        String trajName = isLeft ? "LeftSweep" : "RightSweep";

        return Commands.sequence(
            Commands.sequence(
                homingCmd(),
                Commands.waitUntil(() -> m_shooter.m_isTurretHomed),  //TODO: should probably change this to check if is aiming at target (hub)
                shootWithTimeout(kShooterAutonCloseRPS, 2)
            ),
            Commands.deadline(
                runTraj(trajName, AutonK.kOneSweepMaxTime),
                Commands.sequence(
                    Commands.waitSeconds(2.17),
                    m_superstructure.intake(() -> false).withTimeout(7.5),
                    m_superstructure.unjamCmd()
                )
            ),
            Commands.parallel(
                shootWithTimeout(kShooterAuton_EndSweep_RPS ,12),
                Commands.sequence(
                    Commands.waitSeconds(6),
                    m_superstructure.shimmy()
                )
            )
        ).withName(trajName);
    }

    public Command rightOneSweep() {
        return preload_oneSweep(false);
    }

    public Command leftOneSweep() {
        return preload_oneSweep(true);
    }

    //OVERALL TODO: clean up code and combine no preload w/ preload (and left/right) into one method
    //TODO: find better name for this
    public Command oneCycleGoInNow(boolean left) {
        String path = left ? "LeftSweep" : "RightSweep";
        return Commands.sequence(
            Commands.parallel(
                homingCmd().andThen(
                    Commands.waitSeconds(0.75),
                    m_superstructure.intake(() -> false).withTimeout(7.5)
                ),
                runTraj(path, AutonK.kOneSweepMaxTime)
            ),
            Commands.deadline(
                shootWithTimeout(kShooterAuton_EndSweep_RPS, 12),
                Commands.sequence(
                    Commands.waitSeconds(5),
                    m_superstructure.shimmy()
                )  
            )
        ).withName(path);
    }

    public Command rightTwoSweep() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry("RightSweep"),
                m_superstructure.activateOuttake(kShooterAutonCloseRPS).withTimeout(2)
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
                shootWithTimeout(kShooterAutonCloseRPS, 2)
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
                shootWithTimeout(kShooterAutonCloseRPS, 2)
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

