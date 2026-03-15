package frc.robot.autons;

import static frc.robot.Constants.ShooterK.kShooterAutonCloseRPS;
import static frc.robot.Constants.ShooterK.kShooterAuton_EndSweep_RPS;
import static frc.robot.Constants.AutonK.*;

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
import frc.util.WaltLogger.BooleanLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructure;
    public final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Swerve m_swerve;

    private final DoubleLogger log_autonState = WaltLogger.logDouble(kLogTab, "State");

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
            m_superstructure.activateOuttake(() -> speed).withTimeout(seconds),
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

    private Command preload_oneSweep(boolean isLeft) {
        String trajName = isLeft ? AutonK.kLeftSweepPathName : AutonK.kRightSweepPathName;

        return Commands.sequence(
            tp("preload.sequence.START"),
            logState(0),
            Commands.sequence(
                tp("preload.homing.START"),
                homingCmd().withName("HomingCmd"),
                tp("preload.homing.END"),
                logState(1),
                tp("preload.waitTurretHomed.START"),
                Commands.waitUntil(m_shooter.turretHomedSupp),  //TODO: should probably change this to check if is aiming at target (hub)
                tp("preload.waitTurretHomed.END"),
                logState(2),
                tp("preload.shoot.START"),
                shootWithTimeout(kShooterAutonCloseRPS, 2).withName("ShootingCmd"),
                tp("preload.shoot.END")
            ),
            logState(3),
            tp("preload.deadline.START"),
            Commands.deadline(
                runTraj(trajName, AutonK.kOneSweepMaxTime).withName("RunPath" + trajName),
                Commands.sequence(
                    logState(3.1),
                    tp("preload.deadline.waitSeconds.START"),
                    Commands.waitSeconds(2.17),
                    tp("preload.deadline.waitSeconds.END"),
                    logState(3.2),
                    tp("preload.deadline.intake.START"),
                    m_superstructure.intake(() -> false).withTimeout(7.5),
                    tp("preload.deadline.intake.END"),
                    logState(3.3),
                    tp("preload.deadline.unjam.START"),
                    m_superstructure.unjamCmd(() -> false),
                    tp("preload.deadline.unjam.END")
                )
            ),
            tp("preload.deadline.END"),
            logState(4),
            tp("preload.endShoot.START"),
            Commands.parallel(
                shootWithTimeout(kShooterAuton_EndSweep_RPS, 12),
                Commands.sequence(
                    logState(4.1),
                    Commands.waitSeconds(6),
                    logState(4.2),
                    tp("preload.shimmy.START"),
                    m_superstructure.shimmy(),
                    tp("preload.shimmy.END")
                )
            ),
            tp("preload.endShoot.END")
        ).withName(trajName);
    }

    public Command rightOneSweep() {
        return preload_oneSweep(false);
    }

    public Command leftOneSweep() {
        return preload_oneSweep(true);
    }

    public Command firstSweep_NoPreload(boolean left, boolean optimized) {
        // String regPath = left ? AutonK.kLeftSweepPathName : AutonK.kRightSweepPathName;
        String path = left ? AutonK.kLeftOptimizedSweepPathName : AutonK.kRightOptimizedSweepPathName;

        // String path = optimized ? optPath : regPath;
        
        return Commands.sequence(
            tp("goInNow.sequence.START"),
            logState(0),
            tp("goInNow.parallel.START"),
            Commands.parallel(
                homingCmd().andThen(
                    tp("goInNow.homing.END"),
                    logState(0.1),
                    // Commands.waitSeconds(0.0001),
                    logState(0.2),
                    tp("goInNow.intake.START"),
                    m_superstructure.intake(() -> false).withTimeout(AutonK.kIntakeTimeout).asProxy(),
                    tp("goInNow.intake.END")
                ),
                Commands.sequence(
                    tp("goInNow.traj.START"),
                    runTraj(path, AutonK.kOneSweepMaxTime),
                    tp("goInNow.traj.END"),
                    logState(0.99)
                )
            ),
            tp("goInNow.parallel.END"),
            logState(1),
            tp("goInNow.shootDeadline.START"),
            Commands.deadline(
                shootWithTimeout(kShooterAuton_EndSweep_RPS, AutonK.kShootingTimeout).asProxy(),
                Commands.sequence(
                    logState(1.1),
                    Commands.waitSeconds(3.5),
                    logState(1.2),
                    tp("goInNow.retractIntake.START"),
                    m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED).asProxy(),
                    tp("goInNow.retractIntake.END")
                )
            ),
            tp("goInNow.shootDeadline.END"),
            logState(2)
        ).withName(path);
    }

    public Command rightTwoSweep(boolean left) {
        String path = left ? AutonK.kLeftTwoSweepName : AutonK.kRightTwoSweepName;

        return Commands.sequence(
            firstSweep_NoPreload(left, true),
            Commands.parallel(
                Commands.sequence(
                    Commands.waitSeconds(2),
                    m_superstructure.intake(() -> false).asProxy().withTimeout(4)
                ),
                runTraj(path, AutonK.kTwoSweepMaxTime)
            ),
            Commands.deadline(
                shootWithTimeout(kShooterAuton_EndSweep_RPS, AutonK.kShootingTimeout).asProxy(),
                Commands.sequence(
                    Commands.waitSeconds(2.75),
                    m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED).asProxy()
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
                    m_superstructure.activateOuttake(() -> ShooterK.kShooterRPS).withTimeout(2)
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
                    m_superstructure.activateOuttake(() -> ShooterK.kShooterRPS).withTimeout(2),
                    Commands.waitSeconds(0.5),
                    m_intake.setIntakeArmPosCmd(IntakeArmPosition.SAFE),
                    Commands.waitSeconds(0.5),
                    m_superstructure.intake(() -> false).withTimeout(2)
                )
            )
        );
    }
}
