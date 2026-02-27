package frc.robot.vision;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.VisionUtil;
import frc.robot.Robot;

import static frc.robot.Constants.VisionK;
import static frc.robot.Constants.FieldK;

public class WaltCamera extends PhotonCamera {
    /* CLASS VARIABLES */
    private static final int kGlobalFpsLimit = 5;

    public static final List<WaltCamera> AllCameras = Collections.unmodifiableList(Arrays.asList(
        // new WaltCamera("FrontLeft", VisionK.kFrontLeftCTR, VisionUtil.SimCamProps("ThriftyCam", 0, 0, 0, 0)),
        new WaltCamera("FrontRight", VisionK.kFrontRightRTC, VisionUtil.SimCamProps("ThriftyCam", 0, 0, 0, 0)),
        // new WaltCamera("BackLeft", VisionK.kBackLeftCTR, VisionUtil.SimCamProps("ThriftyCam", 0, 0, 0, 0)),
        new WaltCamera("BackRight", VisionK.kBackRightRTC, VisionUtil.SimCamProps("ThriftyCam", 0, 0, 0, 0))
    ));

    //---SIM
    private final PhotonCameraSim m_sim;
    public static final VisionSim m_visionSim = new VisionSim();

    private final Transform3d m_robotToCam;
    private final PhotonPoseEstimator m_estimator;
    
    private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 6.24);
    private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 6.24);

    private final StructPublisher<Pose2d> log_camPose;
    private final StructPublisher<Transform3d> log_camTransform;
    private final DoubleArrayPublisher log_stdDevs;

    private Matrix<N3, N1> m_curStdDevs;

    /* CONSTRUCTOR */
    public WaltCamera(String cameraName, Transform3d robotToCam, SimCameraProperties simCamProperties) {
        super(cameraName);
        m_robotToCam = robotToCam;
        m_estimator = new PhotonPoseEstimator(FieldK.kTagLayout, m_robotToCam);

        final String ntPrefix = "Vision/" + cameraName + "/";
        log_camPose = NetworkTableInstance.getDefault()
            .getStructTopic(ntPrefix + "estRobotPose", Pose2d.struct).publish();
        log_camTransform = NetworkTableInstance.getDefault()
            .getStructTopic(ntPrefix + "transform", Transform3d.struct).publish();
        log_stdDevs = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic(ntPrefix + "/stdDevs").publish();

        log_camTransform.accept(robotToCam);

        // Simulation
        if (Robot.isSimulation()) {
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible targets.
            m_sim = new PhotonCameraSim(this, simCamProperties);
            // Add the simulated camera to view the targets on this simulated field.
            m_visionSim.addCamera(m_sim, robotToCam);

            m_sim.enableDrawWireframe(true);
        } else m_sim = null; //temp solution to make code compiled.
    }

    /* METHODS */
    public void takeOutputSnapshot() {
        super.takeOutputSnapshot();
    }

    public void takeInputSnapshot() {
        super.takeInputSnapshot();
    }

    public void takeBothSnapshots() {
        takeInputSnapshot();
        takeOutputSnapshot();
        System.out.println(super.getName() + ": BothSnapshot");
    }

    public static void setFpsLimit(boolean limited) {
        for (var cam : AllCameras) {
            cam.setFPSLimit(limited ? kGlobalFpsLimit : -1); 
        }
    }

    public static Command setFpsLimitCmd(boolean limited) {
        return Commands.runOnce(() -> {
            for (var cam : AllCameras) {
                cam.setFPSLimit(limited ? kGlobalFpsLimit : -1); 
            }
        }).andThen(Commands.print("FPS Limit: " + limited));
    }
    
    /* VISION ESTIMATION METHODS */
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
        List<PhotonPipelineResult> unreadCameraResults = this.getAllUnreadResults();

        for (var change : unreadCameraResults) {
            visionEst = m_estimator.estimateCoprocMultiTagPose(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
            log_stdDevs.accept(m_curStdDevs.getData());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                m_visionSim.getSimDebugField()
                                    .getObject(this.getName() + "VisionEstimation")
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
            m_curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = m_estimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
                m_curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                else if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                m_curStdDevs = estStdDevs;
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
        return m_curStdDevs;
    }
}