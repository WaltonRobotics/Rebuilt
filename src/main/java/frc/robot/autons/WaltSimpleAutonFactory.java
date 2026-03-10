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

    private Command tp(String message) {
        return WaltLogger.timedPrintCmd(message);
    }    

    public Command preheater() {
        return Commands.sequence(
            tp("preheat.START"),
            runTraj("PreHeat", 1.0),
            tp("preheat.END")
        );
    }

    private Command waitTurretHomedCmd() {
        return Commands.waitUntil(m_shooter.turretHomedSupp);
    }

    private Command waitIntakeHomedCmd() {
        return Commands.waitUntil(m_intake.intakeHomedSupp);
    }

    private Command homingCmd() {
        return Commands.parallel(
            Commands.sequence(tp("turretHoming.START"), waitTurretHomedCmd(), tp("turretHoming.END")),
            Commands.sequence(tp("intakeArmHoming.START"), waitIntakeHomedCmd(), tp("intakeArmHoming.END"))
        ).withTimeout(5);
    }

    private Command shootWithTimeout(AngularVelocity speed, double seconds) {
        return Commands.sequence(
            tp("shootWithTimeout.START(" + speed + "," + seconds + ")"),
            m_superstructure.activateOuttake(speed).withTimeout(seconds),
            tp("shootWithTimeout.END")
        );
    }

    private Command runTraj(String name, double timeoutSecs) {
        return Commands.sequence(
            tp("runTraj.START(" + name + ")"),
            m_autoFactory.trajectoryCmd(name).withTimeout(timeoutSecs),
            tp("runTraj.END(" + name + ")"),
            m_swerve.xBrake(),
            tp("xBrake.END")
        );
    }

    public Command noPreload_oneSweep(boolean left) {
        String path = left ? AutonK.kLeftSweepPathName : AutonK.kRightSweepPathName;

        return Commands.sequence(
            Commands.parallel(
                homingCmd().andThen(
                    Commands.waitSeconds(0.0001),
                    m_superstructure.intake(() -> false).withTimeout(7.5)
                ),
                Commands.sequence(
                    runTraj(path, AutonK.kOneSweepMaxTime)
                )
            ),
            Commands.deadline(
                shootWithTimeout(kShooterAuton_EndSweep_RPS, 12),
                Commands.sequence(
                    Commands.waitSeconds(3),
                    m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
                )
            )
        ).withName(path);
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

    private Command preload_oneSweep(boolean isLeft) {
        String trajName = isLeft ? AutonK.kLeftSweepPathName : AutonK.kRightSweepPathName;

        return Commands.sequence(
            Commands.sequence(
                homingCmd().withName("HomingCmd"),
                Commands.waitUntil(m_shooter.turretHomedSupp),
                shootWithTimeout(kShooterAutonCloseRPS, 2).withName("ShootingCmd")
            ),
            Commands.deadline(
                runTraj(trajName, AutonK.kOneSweepMaxTime).withName("RunPath" + trajName),
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
        ).withName(trajName));
    }
}
