package frc.robot;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionK;

public class Detection {
    private final PhotonCamera camera = new PhotonCamera(VisionK.kCamera1CamName); //probably will be intake camera

    private List<PhotonTrackedTarget> targetList = new LinkedList<>();

    /**
     * loops through the pipeline results and adds target data results to a list of targetData
     */
    private void processCameraData() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            Optional<PhotonTrackedTarget> bestTarget = Optional.of(result.getBestTarget());

            if (bestTarget.isPresent()) {
                targetList.add(bestTarget.get()); 
            }
        }
    }

    /**
     * sort and find closest target
     */
    public PhotonTrackedTarget getClosestObject() {
        processCameraData();
        PhotonTrackedTarget closestTarget = new PhotonTrackedTarget();

        if (targetList.size() > 0) {
            closestTarget = targetList.get(0);
            double minDistance = 
                PhotonUtils.calculateDistanceToTargetMeters(
                    0,  //dummy (move ts to constants)
                    0,  //dummy (move ts to constants)
                    0,  //dummy (move ts to constants)
                    closestTarget.pitch);
                    
            for (PhotonTrackedTarget target : targetList) {
                double distance = 
                    PhotonUtils.calculateDistanceToTargetMeters(
                        0,  //dummy (move ts to constants)
                        0,  //dummy (move ts to constants)
                        0,  //dummy (move ts to constants)
                        target.pitch);

                if (distance < minDistance) {
                    closestTarget = target;
                    minDistance = distance;
                }
            }
        }

        return closestTarget;
    }

    public Pose3d targetToPose(PhotonTrackedTarget target) {
        //all values in this class are dummies and should be looked over

        Transform3d robotToCamera = 
            new Transform3d(
                Units.inchesToMeters(-1.572730),
                Units.inchesToMeters(5.775637),
                Units.inchesToMeters(37.497937),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30),
                    Units.degreesToRadians(186.370246)));
        
        Pose2d originalRobotPose = new Pose2d(0, 0, new Rotation2d());

        double targetPitch = -Math.toRadians(target.getPitch());
        double targetYaw = -Math.toRadians(target.getYaw());

        Pose3d cameraPoseZeroed = new Pose3d(originalRobotPose).transformBy(robotToCamera);

        Pose3d cameraPointingAtTarget =
            cameraPoseZeroed.transformBy(
                new Transform3d(0, 0, 0, new Rotation3d(0.0, targetPitch, targetYaw)));

        Pose3d targetPose =
            cameraPointingAtTarget.plus(new Transform3d(1.0, 0.0, 0.0, new Rotation3d()));

        return targetPose;
    }
}
