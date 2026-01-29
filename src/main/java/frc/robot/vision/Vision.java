package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Robot;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.FieldK.kTagLayout;

public class Vision {
    //TODO: research about april tag pipeline code (see reefscape at-cmp)
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;

    // Simulation
    private PhotonCameraSim m_cameraSim;
    private final VisionSim m_visionSim;
    private final String m_simVisualName;

    private String m_cameraName;
    private final Transform3d m_roboToCam;

    //Constants
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 6.24);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 6.24);

    //Ignore trench April Tags
    public final HashSet<Integer> excludedAprilTagsID = new HashSet<> (Arrays.asList(0, 8, 10, 12, 13, 14, 20, 23));

    private final StructPublisher<Pose2d> log_camPose;
    private final DoubleArrayPublisher log_stdDevs;

    public Vision(String cameraName, String simVisualName, Transform3d roboToCam, VisionSim visionSim, SimCameraProperties simCameraProperties) {
        m_cameraName = cameraName;
        m_camera = new PhotonCamera(m_cameraName);
        m_roboToCam = roboToCam;
        m_simVisualName = simVisualName;
        m_visionSim = visionSim;

        photonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, roboToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        log_camPose = NetworkTableInstance.getDefault()
            .getStructTopic("Vision/" + cameraName + "/estRobotPose", Pose2d.struct).publish();
        log_stdDevs = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("Vision/" + cameraName + "/stdDevs").publish(); 

        // Simulation
        if (Robot.isSimulation()) {
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible targets.
            m_cameraSim = new PhotonCameraSim(m_camera, simCameraProperties);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(m_cameraSim, m_roboToCam);

            m_cameraSim.enableDrawWireframe(true);
        }
    }

    public void takeOutputSnapshot() {
        m_camera.takeOutputSnapshot();
    }

    public void takeInputSnapshot() {
        m_camera.takeInputSnapshot();
    }

    public void takeBothSnapshots() {
        takeInputSnapshot();
        takeOutputSnapshot();
        System.out.println(m_cameraName + ": BothSnapshot");
    }
    
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        List<PhotonPipelineResult> unreadCameraResults = m_camera.getAllUnreadResults();

        for (var change : unreadCameraResults) {
            // Remove excluded April Tags
            change.getTargets().removeIf(
                    tgt -> excludedAprilTagsID.contains(tgt.getFiducialId()));

            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
            log_stdDevs.accept(curStdDevs.getData());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                m_visionSim.getSimDebugField()
                                    .getObject(m_simVisualName)
                                    .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            m_visionSim.getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
        }

        if (visionEst.isPresent()) {
            log_camPose.accept(visionEst.get().estimatedPose.toPose2d());
        }

        return visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                else if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}