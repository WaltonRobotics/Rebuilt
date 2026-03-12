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
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructure;
    public final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Swerve m_swerve;

    private final DoubleLogger log_autonState = WaltLogger.logDouble("Auton", "State");

    public WaltSimpleAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake, Shooter shooter, Swerve swerve) {
        m_superstructure = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
        m_shooter = shooter;
        m_swerve = swerve;
        log_autonState.accept(-1.0);
    }

    public static final class WaltPathAndCommand {
        public final String pathName;
        public final Command autonCommand;

        public WaltPathAndCommand(String pathName_, Command autonCommand_) {
            pathName = pathName_;
            autonCommand = autonCommand_;
        }
    }
    private Command logState(double state) {
        return Commands.runOnce(() -> log_autonState.accept(state));
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
        String trajName = isLeft ? AutonK.kLeftSweepPathName : AutonK.kRightSweepPathName;

        return Commands.sequence(
            logState(0),
            Commands.sequence(
                homingCmd().withName("HomingCmd"),
                logState(1),
                Commands.waitUntil(() -> m_shooter.m_isTurretHomed),  //TODO: should probably change this to check if is aiming at target (hub)
                logState(2),
                shootWithTimeout(kShooterAutonCloseRPS, 2).withName("ShootingCmd")
            ),
            logState(3),
            Commands.deadline(
                runTraj(trajName, AutonK.kOneSweepMaxTime).withName("RunPath" + trajName),
                Commands.sequence(
                    logState(3.1),                
                    Commands.waitSeconds(2.17),
                    logState(3.2),
                    m_superstructure.intake(() -> false).withTimeout(7.5),
                    logState(3.3),
                    m_superstructure.unjamCmd()
                )
            ),
            logState(4),
            Commands.parallel(
                shootWithTimeout(kShooterAuton_EndSweep_RPS ,12),
                Commands.sequence(
                    logState(4.1),
                    Commands.waitSeconds(6),
                    logState(4.2),
                    m_superstructure.intakeArmShimmy()
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
        String path = left ? AutonK.kLeftSweepPathName : AutonK.kRightSweepPathName;
        return Commands.sequence(
            logState(0),
            Commands.parallel(
                homingCmd().andThen(
                    logState(0.1),
                    Commands.waitSeconds(0.0001),
                    logState(0.2),
                    m_superstructure.intake(() -> false).withTimeout(7.5)
                ),
                runTraj(path, AutonK.kOneSweepMaxTime)
                    .andThen(logState(0.99))
            ),
            logState(1),
            Commands.deadline(
                shootWithTimeout(kShooterAuton_EndSweep_RPS, 12),
                Commands.sequence(
                    logState(1.1),
                    Commands.waitSeconds(3), // 5
                    logState(1.2),
                    // m_superstructure.shimmy()                   
                    m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
                )  
            ),
            logState(2)
        ).withName(path);
    }

    public Command rightTwoSweep() {
        return Commands.sequence(
            Commands.parallel(
                m_autoFactory.resetOdometry(AutonK.kRightSweepPathName),
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

