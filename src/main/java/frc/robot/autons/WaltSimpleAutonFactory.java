package frc.robot.autons;

import static frc.robot.Constants.ShooterK.kShooterAuton_EndSweep_RPS;
import static frc.robot.Constants.SuperstructureK.kLogTab;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonK;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;
import frc.util.WaltLogger.Pose2dArrayLogger;
import frc.util.WaltLogger.StringLogger;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructure;
    public final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Swerve m_swerve;

    private final DoubleLogger log_autonState = WaltLogger.logDouble(kLogTab, "State");

    // trajectory logger
    private final StringLogger log_trajectoryName = new StringLogger(AutonK.kLogTab, "trajectoryName");
    private final Pose2dArrayLogger log_trajectoryPoses = new Pose2dArrayLogger(AutonK.kLogTab, "trajectoryPoses");

    public WaltSimpleAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake, Shooter shooter, Swerve swerve) {
        m_superstructure = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
        m_shooter = shooter;
        m_swerve = swerve;
        log_autonState.accept(-1.0);
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

    // --- Preheater (stays as Command, runs during disabled) ---

    public AutoRoutine preheater() {
        String path = "PreHeat";
        AutoRoutine routine = m_autoFactory.newRoutine(path);
        AutoTrajectory traj = createTraj(routine, path);

        routine.active().onTrue(
            tp("preheat.START")
            .andThen(
                traj.cmd(),
                tp("preheat.END"),
                m_swerve.xBrakeCmd()
            )
        );

        return routine;
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

    // --- AutoRoutine auton methods ---

    public AutoRoutine firstSweep_NoPreload(boolean left) {
        String path = left ? AutonK.kLeftOptimizedSweepPathName
                           : AutonK.kRightOptimizedSweepPathName;
        AutoRoutine routine = m_autoFactory.newRoutine("firstSweep_" + (left ? "L" : "R"));
        AutoTrajectory traj = createTraj(routine, path);

        // Trajectory starts immediately
        routine.active().onTrue(traj.cmd().withTimeout(AutonK.kOneSweepMaxTime));

        // Homing then intake in parallel with trajectory (no .asProxy() needed)
        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.intake(() -> false).withTimeout(AutonK.kIntakeTimeout)
            )
        );

        // After trajectory: xBrake + shoot
        traj.done().onTrue(Commands.sequence(
            m_swerve.xBrakeCmd(),
            shootWithTimeout(kShooterAuton_EndSweep_RPS, AutonK.kShootingTimeout)
        ));

        // Retract intake 3.5s after trajectory ends (during shoot phase)
        traj.doneDelayed(3.5).onTrue(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        );

        return routine;
    }

    public AutoRoutine preloadAuton() {
        String path = "PreHeat";
        AutoRoutine routine = m_autoFactory.newRoutine("preloadAuton");
        AutoTrajectory traj = createTraj(routine, path);

        routine.active().onTrue(traj.cmd().withTimeout(5));

        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.activateOuttakeShotCalc()
            )
        );

        return routine;
    }

    public AutoRoutine fastOneSweep(boolean left) {
        String path = left ? AutonK.kFastLeftTwoSweepName
                           : AutonK.kFastRightTwoSweepName;
        AutoRoutine routine = m_autoFactory.newRoutine("fastOneSweep_" + (left ? "L" : "R"));
        AutoTrajectory traj = createTraj(routine, path);

        routine.active().onTrue(traj.cmd().withTimeout(6));

        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.intake(() -> false).withTimeout(3.5)
            )
        );

        traj.done().onTrue(Commands.sequence(
            m_swerve.xBrakeCmd(),
            shootWithTimeout(kShooterAuton_EndSweep_RPS, AutonK.kShootingTimeout)
        ));

        traj.doneDelayed(3.5).onTrue(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        );

        return routine;
    }

    public AutoRoutine fastRightTwoSweep_ZigZag() {
        AutoRoutine routine = m_autoFactory.newRoutine("fastRightTwoSweep_ZigZag");
        AutoTrajectory traj1 = createTraj(routine, AutonK.kFastRightTwoSweepName);
        AutoTrajectory traj2 = createTraj(routine, AutonK.kZigzagRightTwo);

        // Phase 1: first sweep + homing/intake
        routine.active().onTrue(traj1.cmd().withTimeout(6));

        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.intake(() -> false).withTimeout(3.5)
            )
        );

        // Transition: xBrake -> shoot -> traj2
        traj1.done().onTrue(Commands.sequence(
            m_swerve.xBrakeCmd(),
            shootWithTimeout(kShooterAuton_EndSweep_RPS, AutonK.kShootingTimeout),
            traj2.cmd().withTimeout(12)
        ));

        traj1.doneDelayed(3.5).onTrue(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        );

        // Phase 2: intake during zigzag (no final shoot)
        traj2.atTime(2).onTrue(
            m_superstructure.intake(() -> false).withTimeout(10)
        );

        traj2.done().onTrue(m_swerve.xBrakeCmd());

        return routine;
    }

    public AutoRoutine fastTwoSweep_Reshoot(boolean left) {
        String path1 = left ? AutonK.kFastLeftTwoSweepName : AutonK.kFastRightTwoSweepName;
        String path2 = left ? AutonK.kReshootLeftTwo : AutonK.kReshootRightTwo;
        AutoRoutine routine = m_autoFactory.newRoutine(
            "fastTwoSweep_Reshoot_" + (left ? "L" : "R"));
        AutoTrajectory traj1 = createTraj(routine, path1);
        AutoTrajectory traj2 = createTraj(routine, path2);

        // Phase 1: first sweep + homing/intake
        routine.active().onTrue(traj1.cmd());

        Trigger stopIntk1Trg = traj1.atTime("stopIntake");
        // Trigger stopIntk2Trg = traj2.atTime("stopIntake");
        Trigger startIntk2Trg = traj2.atTime(0.28);

        stopIntk1Trg.onChange(Commands.print("StopIntk1Trg Change!"));
        // stopIntk2Trg.onChange(Commands.print("StopIntk2Trg Change!"));
        startIntk2Trg.onChange(Commands.print("StartIntk2Trg Change!"));

        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.intake(() -> false)
            ).until(stopIntk1Trg)
        );

        // Transition: xBrake -> shoot -> traj2
        traj1.done().onTrue(
            m_swerve.xBrakeCmd()
        );

        // cope to "stop" superstructure intaking
        traj1.done().onTrue(
            m_intake.setIntakeRollersVelocityCmd(4)
        );

        traj1.doneDelayed(2.5).onTrue(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        );

        traj1.doneFor(AutonK.kShootingTimeout).whileTrue(
            m_superstructure.activateOuttakeShotCalc()
        );

        traj1.doneDelayed(AutonK.kShootingTimeout + 0.04).onTrue(traj2.cmd());

        // Phase 2: intake during reshoot path
        startIntk2Trg.onTrue(
            m_superstructure.intake(() -> false) //.until(stopIntk2Trg)
        );

        // Final: xBrake + shoot after traj2
        // traj2.done().onTrue(m_swerve.xBrakeCmd());

        // traj2.doneFor(AutonK.kShootingTimeout).whileTrue(
        //     m_superstructure.activateOuttakeShotCalc()
        // );

        // traj2.doneDelayed(2.75).onTrue(
        //     m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        // );

        return routine;
    }
}
