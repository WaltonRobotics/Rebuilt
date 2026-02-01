package frc.robot.autoalign;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoAlignK;

public class MovingAutoAlign {

    public static boolean isInTolerance(Pose2d current, Pose2d target, ChassisSpeeds speeds) {
        final Transform2d diff = current.minus(target);
        final double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return MathUtil.isNear(0.0, diff.getRotation().getRadians(), AutoAlignK.kFieldRotationTol.in(Radians)) &&
               MathUtil.isNear(0.0, Math.hypot(diff.getX(), diff.getY()), AutoAlignK.kFieldTranslationTol.in(Meter)) && //Assume that diff.getX/Y() are in meters
               MathUtil.isNear(0.0, speed, AutoAlignK.kFinishVelTol);
    }

    // Placeholder command for moving to the target pose
    public Command moveToPose() {
        return Commands.runOnce(() -> {
            // Implementation for moving to the target pose
        });
    }
}