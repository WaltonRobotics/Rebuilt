package frc.robot.autons;

import static frc.robot.Constants.SuperstructureK.kLogTab;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonK;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose2dArrayLogger;
import frc.util.WaltLogger.StringLogger;

public class WaltAdaptableAutonFactory {
    private final Superstructure m_superstructure;
    public final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Swerve m_swerve;

    // state logger
    private final DoubleLogger log_autonState = WaltLogger.logDouble(kLogTab, "State");

    // trajectory logger
    private final StringLogger log_trajectoryName = new StringLogger(AutonK.kLogTab, "trajectoryName");
    private final Pose2dArrayLogger log_trajectoryPoses = new Pose2dArrayLogger(AutonK.kLogTab, "trajectoryPoses");

    public WaltAdaptableAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake, Shooter shooter, Swerve swerve) {
        m_superstructure = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
        m_shooter = shooter;
        m_swerve = swerve;
    }

    //---UTILITY METHODS

    private Command logState(double state) {
        return Commands.runOnce(() -> log_autonState.accept(state));
    }

    private Command tp(String message) {
        return WaltLogger.timedPrintCmd(message);
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
            m_superstructure.activateOuttakeShotCalc(), //(() -> speed).withTimeout(seconds),
            tp("shootWithTimeout.END")
        );
    }

    //--- AUTOROUTINE TRAJECTORY HELPER

    private AutoTrajectory createTraj(AutoRoutine routine, String name) {
        AutoTrajectory traj = routine.trajectory(name);
        var poses = Choreo.loadTrajectory(name).get().getPoses();
        traj.active().onTrue(Commands.sequence(
            Commands.runOnce(() -> {
                // log_trajectoryName.accept(name);
                // log_trajectoryPoses.accept(poses);
            }),
            tp("traj.START(" + name + ")")
        ));
        traj.done().onTrue(tp("traj.END(" + name + ")"));
        return traj;
    }
}
