package frc.robot.vision;

import static frc.robot.Constants.FieldK.kTagLayout;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

public class VisionSim {
    private final VisionSystemSim m_photonVisionSim;

    public VisionSim() {
        m_photonVisionSim = new VisionSystemSim("main");
        m_photonVisionSim.addAprilTags(kTagLayout);
    }

    public void addCamera(PhotonCameraSim photonCameraSim, Transform3d roboToCam) {
        m_photonVisionSim.addCamera(photonCameraSim, roboToCam);
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        m_photonVisionSim.update(robotSimPose);
    }

    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) {
            m_photonVisionSim.resetRobotPose(pose);
        }
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) {
            return new Field2d();
        }

        return m_photonVisionSim.getDebugField();
    }
}