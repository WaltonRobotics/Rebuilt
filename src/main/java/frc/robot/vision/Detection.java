package frc.robot.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import frc.robot.Constants.VisionK;


public class Detection {
    /* CLASS VARIABLES */
    private final PhotonCamera[] m_cameras = {
        new PhotonCamera(VisionK.kCameras[0].getCameraName()),
        new PhotonCamera(VisionK.kCameras[1].getCameraName()),
        new PhotonCamera(VisionK.kCameras[2].getCameraName()),
        new PhotonCamera(VisionK.kCameras[3].getCameraName())
    };
    private final VisionSim m_visionSim = new VisionSim();

    private List<PhotonTrackedTarget> m_targetList = new LinkedList<>();

    /* METHODS */
    /**
     * loops through the pipeline results and adds target data results to a list of targetData
     */
    private void processCameraData() {
        List<PhotonPipelineResult> results = new LinkedList<>();
        for(PhotonCamera camera : m_cameras) {
            results.addAll(camera.getAllUnreadResults());
        }

        for (PhotonPipelineResult result : results) {
            if (result.hasTargets()) {
                Optional<PhotonTrackedTarget> bestTarget = Optional.of(result.getBestTarget());

                if (bestTarget.isPresent()) {
                    m_targetList.add(bestTarget.get()); 
                }
            }
        }
    }

    /**
     * sort and find closest target
     */
    public PhotonTrackedTarget getClosestObject() {
        processCameraData();
        PhotonTrackedTarget closestTarget = new PhotonTrackedTarget();

        if (Robot.isSimulation()) {
            m_targetList = m_visionSim.simFuelList();
        }

        if (m_targetList.size() > 0) {
            double closestArea = 0;
                    
            for (PhotonTrackedTarget target : m_targetList) {
                double area = target.getArea();

                //closer the target, the larger the area
                if (area > closestArea) {
                    closestTarget = target;
                    closestArea = area;
                }
            }
        }

        return closestTarget;
    }

    public Pose2d targetToPose(Pose2d robotPose, PhotonTrackedTarget fuel) {
        Transform2d fuelTransform = new Transform2d(
            fuel.getBestCameraToTarget().getMeasureX(), fuel.getBestCameraToTarget().getMeasureY(), fuel.getBestCameraToTarget().getRotation().toRotation2d()
        );
        
        Pose2d fuelPose = robotPose.transformBy(fuelTransform);

        return fuelPose;
    }

    public void addFuel(Pose2d destination) {
        m_visionSim.getSimDebugField().getObject("fuel").setPose(destination);
    }
}
