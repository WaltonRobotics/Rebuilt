package frc.robot.subsystems;

import static frc.robot.Constants.FieldK.kBlueGoalLocation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public void update (Pose2d robotPose, ChassisSpeeds robotSpeed) {
        
        //do we need to account for latency in any ways
        Translation2d futurePosition = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
        );

        Translation2d goalLocation = kBlueGoalLocation;
        Translation2d targetVector = goalLocation.minus(futurePosition);
        double dist = targetVector.getNorm();

        // Note: This returns HORIZONTAL velocity component
        double idealHorizontalSpeed = ShooterTable.getSpeed(dist);


    }
}
