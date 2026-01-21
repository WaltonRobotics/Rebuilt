package frc.robot;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionK;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
        PhotonTrackedTarget closestTarget = targetList.get(0);
        double minDistance = 
            PhotonUtils.calculateDistanceToTargetMeters(
                    1,  //dummy (move ts to constants)
                    2,  //dummy (move ts to constants)
                    3,  //dummy (move ts to constants)
                    closestTarget.pitch);
                    
        for (PhotonTrackedTarget target : targetList) {
            double distance = 
                PhotonUtils.calculateDistanceToTargetMeters(
                    1,  //dummy (move ts to constants)
                    2,  //dummy (move ts to constants)
                    3,  //dummy (move ts to constants)
                    target.pitch);

            if (distance < minDistance) {
                closestTarget = target;
                minDistance = distance;
            }
        }

        return closestTarget;
    }

    /**
    * PhotonTrackedTarget to Pose2d
    */
    public Pose2d targetToPose(PhotonTrackedTarget target) {
        Transform3d transform = target.getBestCameraToTarget();
        Pose2d pose = new Pose2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
        return pose;
    }
}
