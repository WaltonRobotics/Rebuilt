package frc.shooting;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.Vision;

public class ShootingUtil {

    Vision camera = new Vision("tempName", null, null, null, null);
    EstimatedRobotPose estimatedRobotPose = camera.get

    Translation2d goalPosition =
}
