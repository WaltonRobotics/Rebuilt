package frc.robot.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

import frc.robot.Robot;
import static frc.robot.Constants.VisionK.*;
import static org.wpilib.units.Units.Rotation;


public class Detection {
    /* CLASS VARIABLES */
    // private final List<WaltCamera> m_cameras = WaltCamera.AllCameras;
    private final WaltCamera m_camera = WaltCamera.m_intakeCam;
    private final VisionSim m_visionSim = new VisionSim();

    private List<PhotonTrackedTarget> m_targetList = new LinkedList<>();

    /* METHODS */
    /**
     * loops through the pipeline results and adds target data results to a list of targetData
     */
    // private void processCameraData() {
    //     List<PhotonPipelineResult> results = new LinkedList<>();
    //     for(PhotonCamera camera : m_cameras) {
    //         results.addAll(camera.getAllUnreadResults());
    //     }

    //     for (PhotonPipelineResult result : results) {
    //         if (result.hasTargets()) {
    //             Optional<PhotonTrackedTarget> bestTarget = Optional.of(result.getBestTarget());

    //             if (bestTarget.isPresent()) {
    //                 m_targetList.add(bestTarget.get()); 
    //             }
    //         }
    //     }
    // }

    private void processCameraData() {
        List<PhotonPipelineResult> results = new LinkedList<>();
        
        results.addAll(m_camera.getAllUnreadResults());

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

    public PhotonTrackedTarget getLargestCluster() {
        m_targetList.clear();
        processCameraData();

        //center b/c want to go to the center whenever toPose-ing, and also best place for comparison
        PhotonTrackedTarget bestClusterCenter = null;
        //weight the cluster based off area
        double maxClusterWeight = 0;
        

        if (!m_targetList.isEmpty()) {
            for (PhotonTrackedTarget primaryTarget : m_targetList) {
                double currentClusterWeight = primaryTarget.getArea();

                for (PhotonTrackedTarget secondaryTarget : m_targetList) {
                    if (primaryTarget == secondaryTarget) continue;

                    //calc the difference in targets (how far they are from each other)
                    double deltaYaw = Math.abs(primaryTarget.yaw - secondaryTarget.yaw);
                    double deltaPitch = Math.abs(primaryTarget.pitch - secondaryTarget.pitch);

                    if (deltaPitch <= kFuelClumpToleranceDeg && deltaYaw <= kFuelClumpToleranceDeg) {
                        currentClusterWeight += secondaryTarget.area;
                    }
                }

                if (currentClusterWeight > maxClusterWeight) {
                    maxClusterWeight = currentClusterWeight;
                    bestClusterCenter = primaryTarget;
                }
            }
        }

        return bestClusterCenter != null ? bestClusterCenter : new PhotonTrackedTarget();
    }

    public double getDistanceToObject(PhotonTrackedTarget target) {
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            kIntakeCTR.getZ(), kFuelDiameterMeters / 2.0, kIntakeCTR.getRotation().getY(), Math.toRadians(target.getPitch()));

        return distanceMeters;
    }

    public Pose2d targetToPose(Pose2d robotPose, PhotonTrackedTarget fuel) {
        double distance = getDistanceToObject(fuel);
        
        Translation2d cameraToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(fuel.yaw));

        Translation2d robotToTargetTranslation = cameraToTargetTranslation.plus(kIntakeCTR.getTranslation().toTranslation2d());

        Transform2d fieldTransform = new Transform2d(robotToTargetTranslation, Rotation2d.fromDegrees(fuel.yaw));

        return robotPose.transformBy(fieldTransform);
    }

    public void simAddFuel(Pose2d destination) {
        m_visionSim.getSimDebugField().getObject("fuel").setPose(destination);
    }
}
