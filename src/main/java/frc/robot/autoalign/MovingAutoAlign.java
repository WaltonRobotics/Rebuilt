package frc.robot.autoalign;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoAlignK;
import frc.robot.Constants.FieldK;
import frc.robot.subsystems.Swerve;

public class MovingAutoAlign {

    //Obtain the correct tag ID based on alliance color, default to blue side
    // public static int towerTagID2 = DriverStation.getAlliance().get().equals(Alliance.Blue) 
    //                             || DriverStation.getAlliance().isEmpty() ? 15 : 31;

    public static int towerTagID = 31;

    public static final Pose2d tagPose = FieldK.kTagLayout.getTagPose(towerTagID).get().toPose2d();

    public static boolean isInTolerance(Pose2d current, Pose2d target, ChassisSpeeds speeds) {
        final Transform2d diff = current.minus(target);
        final double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return MathUtil.isNear(0.0, diff.getRotation().getRadians(), AutoAlignK.kFieldRotationTol.in(Radians)) &&
               MathUtil.isNear(0.0, Math.hypot(diff.getX(), diff.getY()), AutoAlignK.kFieldTranslationTol.in(Meter)) && //Assume that diff.getX/Y() are in meters
               MathUtil.isNear(0.0, speed, AutoAlignK.kFinishVelTol);
    }

    public static Command autoAlignWithIntermediateTransformUntilInTolerances(
            Swerve drivetrain,
            Supplier<Pose2d> target, 
            Supplier<Transform2d> intermediateTransform) {
        return autoAlignWithIntermediatePoseUntilInTolerances(
            drivetrain, 
            target, 
            () -> target.get().transformBy(intermediateTransform.get()));
    }

    public static Command autoAlignWithIntermediatePoseUntilInTolerances(
            Swerve drivetrain,
            Supplier<Pose2d> target,
            Supplier<Pose2d> intermediate) {
        return moveToPoseUntilInTolerances(drivetrain, intermediate, ChassisSpeeds::new, () -> AutoAlignK.kXYConstraints)
            .andThen(moveToPoseUntilInTolerances(drivetrain, target, ChassisSpeeds::new, () -> AutoAlignK.kXYConstraints));
    }

    public static Command moveToPoseUntilInTolerances(
            Swerve swerve,
            Supplier<Pose2d> target,
            Supplier<ChassisSpeeds> speedsModifier,
            Supplier<TrapezoidProfile.Constraints> xyConstraints) {
        return moveToPose(swerve, target, speedsModifier, xyConstraints)
            .until(() -> isInTolerance(swerve.getState().Pose, target.get(), swerve.getState().Speeds));
    }

    // Placeholder command for moving to the target pose
    public static Command moveToPose(
        Swerve swerve,
        Supplier<Pose2d> target,
        Supplier<ChassisSpeeds> speedsModifier,
        Supplier<TrapezoidProfile.Constraints> constraints
    ) {
        final Pose2d cachedTarget[] = {Pose2d.kZero};
        // interestingly no kD in the heading controller
        final ProfiledPIDController headingController =
            // assume we can accelerate to max in 2/3 of a second
            new ProfiledPIDController(
                AutoAlignK.kThetaKP, 0.0, 0.0, 
                AutoAlignK.kThetaConstraints);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        // ok, use passed constraints on X controller
        final ProfiledPIDController vxController =
            new ProfiledPIDController(AutoAlignK.kXKP, 0.01, 0.015, constraints.get());
        // use constraints from constants for y controller?
        // why define them with different constraints?? it's literally field relative
        // the difference in x and y dimensions almost definitely do not mean anything to robot movement
        final ProfiledPIDController vyController =
            new ProfiledPIDController(AutoAlignK.kYKP, 0.01, 0.015, constraints.get());

        // this is created at trigger binding, not created every time the command is scheduled
        final SwerveRequest.ApplyFieldSpeeds swreq_driveFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

        return Commands.runOnce(
            () -> {                
                cachedTarget[0] = target.get();

                SwerveDriveState curState = swerve.getState();
                Pose2d curPose = curState.Pose;
                ChassisSpeeds fieldRelativeChassisSpeeds = Swerve.getFieldRelativeChassisSpeeds(curState);
                // for some reason only do logging in simulation?
                // very smart of them to cache whether the robot is in simulation though rather than
                // checking every time though
                // reset profiled PIDs to have the correct speeds
                // we can likely use the Swerve::getVelocityFieldRelative i stuck in there previously
                // stolen from someone elses code
                // this code sets all the setpoints of the controllers

                headingController.reset(
                    curPose.getRotation().getRadians(),
                    fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
                vxController.reset(
                    curPose.getX(), fieldRelativeChassisSpeeds.vxMetersPerSecond);
                vyController.reset(
                    curPose.getY(), fieldRelativeChassisSpeeds.vyMetersPerSecond);
            })
        .andThen(
            // so does this keep running over and over again?
            // i assume it has to make sure that the speeds actually update as
            swerve.applyRequest(
                () -> {
                // get difference between target pose and current pose
                // (this is the transform that maps current pose to target pose)
                // this is only used for tolerances right here.
                final Pose2d curPose = swerve.getState().Pose;
                final Transform2d diff = curPose.minus(cachedTarget[0]);
                final ChassisSpeeds speeds =
                    // for some reason not using tolerance constants?? who knows why
                    MathUtil.isNear(0.0, diff.getX(), Units.inchesToMeters(0.75))
                        && MathUtil.isNear(0.0, diff.getY(), Units.inchesToMeters(0.75))
                        && MathUtil.isNear(0.0, diff.getRotation().getDegrees(), 0.5)
                    // there is no case in code where speedsModifier is nonzero
                    ? new ChassisSpeeds().plus(speedsModifier.get())
                    : new ChassisSpeeds(
                        // these add the setpoint to velocity for some reason?
                        // i just trust that they know how motion profiles work better
                        // than i do
                        // also why do they include the goal in every call? they shouldn't
                        // have to
                        vxController.calculate(
                                curPose.getX(), cachedTarget[0].getX())
                            + vxController.getSetpoint().velocity,
                        vyController.calculate(
                                curPose.getY(), cachedTarget[0].getY())
                            + vyController.getSetpoint().velocity,
                        headingController.calculate(
                                curPose.getRotation().getRadians(),
                                cachedTarget[0].getRotation().getRadians())
                            + headingController.getSetpoint().velocity)
                        // again there is no case existing in code speedsModifier is nonzero
                    .plus(speedsModifier.get());
                    return swreq_driveFieldSpeeds.withSpeeds(speeds);
                    }));
    }
}