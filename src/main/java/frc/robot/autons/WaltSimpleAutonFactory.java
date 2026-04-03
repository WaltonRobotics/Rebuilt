package frc.robot.autons;

import static frc.robot.Constants.SuperstructureK.kLogTab;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonK;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeArmPosition;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.util.WaltLogger;
import frc.util.WaltLogger.DoubleLogger;

public class WaltSimpleAutonFactory {
    private final Superstructure m_superstructure;
    public final AutoFactory m_autoFactory;
    private final Intake m_intake;
    private final Swerve m_drivetrain;

    private final DoubleLogger log_autonState = WaltLogger.logDouble(kLogTab, "State");

    public WaltSimpleAutonFactory(Superstructure superstructure, AutoFactory autoFactory, Intake intake, Swerve swerve) {
        m_superstructure = superstructure;
        m_autoFactory = autoFactory;
        m_intake = intake;
        m_drivetrain = swerve;
        log_autonState.accept(-1.0);
    }

    // --- Utility methods ---

    private Command tp(String message) {
        return WaltLogger.timedPrintCmd(message);
    }

    private Command waitTurretHomedCmd() {
        return Commands.waitUntil(() -> true);
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
                m_drivetrain.xBrakeCmd()
            )
        );

        return routine;
    }

    // --- Autoroutine trajectory helper ---

    private AutoTrajectory createTraj(AutoRoutine routine, String name) {
        AutoTrajectory traj = routine.trajectory(name);
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
    public AutoRoutine fastTwoSweep_Sweep(boolean left) {
        String path1 = left ? AutonK.kFastLeftSweep : AutonK.kFastRightSweep;
        String path2 = left ? AutonK.kSweepLeftTwo : AutonK.kSweepRightTwo;
        AutoRoutine routine = m_autoFactory.newRoutine(
            "fastTwoSweep_Reshoot_" + (left ? "L" : "R"));
        AutoTrajectory traj1 = createTraj(routine, path1);
        AutoTrajectory traj2 = createTraj(routine, path2);

        // Phase 1: first sweep + homing/intake
        routine.active().onTrue(traj1.cmd());

        Trigger stopIntk1Trg = traj1.atTime("stopIntake");
        Trigger startIntk2Trg = traj2.atTime(0.28);

        stopIntk1Trg.onChange(Commands.print("StopIntk1Trg Change!"));
        startIntk2Trg.onChange(Commands.print("StartIntk2Trg Change!"));

        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.intake(() -> false)
            ).until(stopIntk1Trg)
        );

        // Transition: xBrake -> shoot -> traj2
        traj1.done().onTrue(
            m_drivetrain.xBrakeCmd()
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
            m_superstructure.intake(() -> false));

        return routine;
    }

    //TODO: THIS LOGIC NEEDS TO CHANGE
    /**
     * 2 full cycle auto routine
     * <p> (goes to neutral zone, picks up balls, goes back to alliance zone to shoot them) x2
     * @param left
     * @return
     */
    public AutoRoutine fastTwoSweep_Reshoot(boolean left) {
        String path1 = left ? AutonK.kFastLeftSweep : AutonK.kFastRightSweep;
        String path2 = left ? AutonK.kReshootLeftTwo : AutonK.kReshootRightTwo;
        AutoRoutine routine = m_autoFactory.newRoutine(
            "fastTwoSweep_Reshoot_" + (left ? "L" : "R"));
        AutoTrajectory traj1 = createTraj(routine, path1);
        AutoTrajectory traj2 = createTraj(routine, path2);

        // Phase 1: first sweep + homing/intake
        routine.active().onTrue(traj1.cmd());

        Trigger stopIntk1Trg = traj1.atTime("stopIntake");
        Trigger stopIntk2Trg = traj2.atTime("stopIntake");
        Trigger startIntk2Trg = traj2.atTime(0.28);

        stopIntk1Trg.onChange(Commands.print("StopIntk1Trg Change!"));
        stopIntk2Trg.onChange(Commands.print("StopIntk2Trg Change!"));
        startIntk2Trg.onChange(Commands.print("StartIntk2Trg Change!"));

        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.intake(() -> false)
            ).until(stopIntk1Trg)
        );

        // Transition: xBrake -> shoot -> traj2
        traj1.done().onTrue(
            m_drivetrain.xBrakeCmd()
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
            m_superstructure.intake(() -> false).until(stopIntk2Trg)
        );

        // Final: xBrake + shoot after traj2
        traj2.done().onTrue(m_drivetrain.xBrakeCmd());

        traj2.doneFor(AutonK.kShootingTimeout).whileTrue(
            m_superstructure.activateOuttakeShotCalc()
        );

        traj2.doneDelayed(2.75).onTrue(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        );

        return routine;
    }

    /**
     * one sweep auton path that complements an auton ON THE SAME SIDE from a trench bot
     * @param left
     * @return
     */
    public AutoRoutine oneTrenchSweep(boolean left) {
        String path = left ? AutonK.kTrenchLeftSweep : AutonK.kTrenchRightSweep;
        AutoRoutine routine = m_autoFactory.newRoutine("oneTrenchSweep_" + (left ? "L" : "R"));
        AutoTrajectory traj = createTraj(routine, path);

        Trigger stopIntakeTrg = traj.atTime("stopIntake");

        routine.active().onTrue(traj.cmd().withTimeout(8));

        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.intake(() -> false)
            ).until(stopIntakeTrg)
        );

        traj.done().onTrue(
            m_drivetrain.xBrakeCmd()
        );

        traj.doneFor(AutonK.kShootingTimeout).whileTrue(
            m_superstructure.activateOuttakeShotCalc()
        );

        traj.doneDelayed(2.5).onTrue(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        );

        return routine;
    }

    /**
     * one sweep auton, going to the depot after the first sweep, and then heading back to the neutral zone + coming back to reshoot
     * <p> once the first sweep traj ends, the robot starts shooting and continues to shoot until auton ends
     * @return
     */
    public AutoRoutine oneSweepToDepot() {
        String path1 = AutonK.kFastLeftSweep;
        String path2 = "LeftSweepTwo_Depot_copy1"; //AutonK.kDepotLeftTwo;
        String path3 = AutonK.kReshootLeftTwo;
        AutoRoutine routine = m_autoFactory.newRoutine("oneSweepToDepot");
        AutoTrajectory traj1 = createTraj(routine, path1);
        AutoTrajectory traj2 = createTraj(routine, path2);
        AutoTrajectory traj3 = createTraj(routine, path3);

        Trigger stopIntk1Trg = traj1.atTime("stopIntake");
        Trigger startIntk1Trg = traj2.atTime("startIntake");
        Trigger retractIntakeTrg = traj2.atTime("retractIntake");
        Trigger startIntk2Trg = traj3.atTime("startIntake");
        Trigger stopIntk2Trg = traj3.atTime("stopIntake");

        routine.active().onTrue(
            homingCmd().andThen(
                m_superstructure.intake(() -> false)
            ).until(stopIntk1Trg)
        );

        // cope to "stop" superstructure intaking
        traj1.done().onTrue(
            m_intake.setIntakeRollersVelocityCmd(4)
        );

        traj1.done().onTrue(
            m_superstructure.activateOuttakeShotCalc()
        );

        traj1.done().onTrue(traj2.cmd());

        startIntk1Trg.onTrue(
            m_superstructure.intake(() -> false)
        );

        retractIntakeTrg.onTrue(
            m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
        );

        traj2.done().onTrue(traj3.cmd());

        startIntk2Trg.onTrue(
            m_superstructure.intake(() -> false).until(stopIntk2Trg)
        );

        traj3.done().onTrue(
            m_superstructure.activateOuttakeShotCalc()
        );

        return routine;
    }

    // --- UNUSED AutoRoutine auton methods ---
    //TODO: should confirm if these even need to exist

    // public AutoRoutine fastOneSweep(boolean left) {
    //     String path = left ? AutonK.kFastLeftTwoSweepName
    //                        : AutonK.kFastRightTwoSweepName;
    //     AutoRoutine routine = m_autoFactory.newRoutine("fastOneSweep_" + (left ? "L" : "R"));
    //     AutoTrajectory traj = createTraj(routine, path);

    //     routine.active().onTrue(traj.cmd().withTimeout(6));

    //     routine.active().onTrue(
    //         homingCmd().andThen(
    //             m_superstructure.intake(() -> false).withTimeout(3.5)
    //         )
    //     );

    //     traj.done().onTrue(Commands.sequence(
    //         m_swerve.xBrakeCmd(),
    //         shootWithTimeout(kShooterAuton_EndSweep_RPS, AutonK.kShootingTimeout)
    //     ));

    //     traj.doneDelayed(3.5).onTrue(
    //         m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
    //     );

    //     return routine;
    // }

    // public AutoRoutine fastRightTwoSweep_ZigZag() {
    //     AutoRoutine routine = m_autoFactory.newRoutine("fastRightTwoSweep_ZigZag");
    //     AutoTrajectory traj1 = createTraj(routine, AutonK.kFastRightTwoSweepName);
    //     AutoTrajectory traj2 = createTraj(routine, AutonK.kZigzagRightTwo);

    //     // Phase 1: first sweep + homing/intake
    //     routine.active().onTrue(traj1.cmd().withTimeout(6));

    //     routine.active().onTrue(
    //         homingCmd().andThen(
    //             m_superstructure.intake(() -> false).withTimeout(3.5)
    //         )
    //     );

    //     // Transition: xBrake -> shoot -> traj2
    //     traj1.done().onTrue(Commands.sequence(
    //         m_swerve.xBrakeCmd(),
    //         shootWithTimeout(kShooterAuton_EndSweep_RPS, AutonK.kShootingTimeout),
    //         traj2.cmd().withTimeout(12)
    //     ));

    //     traj1.doneDelayed(3.5).onTrue(
    //         m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
    //     );

    //     // Phase 2: intake during zigzag (no final shoot)
    //     traj2.atTime(2).onTrue(
    //         m_superstructure.intake(() -> false).withTimeout(10)
    //     );

    //     traj2.done().onTrue(m_swerve.xBrakeCmd());

    //     return routine;
    // }

    // public AutoRoutine firstSweep_NoPreload(boolean left) {
    //     String path1 = left ? AutonK.kFastLeftTwoSweepName : AutonK.kFastRightTwoSweepName;
    //     AutoRoutine routine = m_autoFactory.newRoutine("firstSweep_" + (left ? "L" : "R"));
    //     AutoTrajectory traj = createTraj(routine, path);

    //     // Trajectory starts immediately
    //     routine.active().onTrue(traj.cmd().withTimeout(AutonK.kOneSweepMaxTime));

    //     // Homing then intake in parallel with trajectory (no .asProxy() needed)
    //     routine.active().onTrue(
    //         homingCmd().andThen(
    //             m_superstructure.intake(() -> false).withTimeout(AutonK.kIntakeTimeout)
    //         )
    //     );

    //     // After trajectory: xBrake + shoot
    //     traj.done().onTrue(Commands.sequence(
    //         m_swerve.xBrakeCmd(),
    //         shootWithTimeout(kShooterAuton_EndSweep_RPS, AutonK.kShootingTimeout)
    //     ));

    //     // Retract intake 3.5s after trajectory ends (during shoot phase)
    //     traj.doneDelayed(3.5).onTrue(
    //         m_intake.setIntakeArmPosCmd(IntakeArmPosition.RETRACTED)
    //     );

    //     return routine;
    // }
}
