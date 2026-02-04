package frc.robot.vision;

import static frc.robot.Constants.FieldK.kTagLayout;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

    public List<PhotonTrackedTarget> addFuel() {
        List<PhotonTrackedTarget> simTargetList = new LinkedList<>();

        Rotation3d fuelRotation = new Rotation3d(10, 30, 45);

        PhotonTrackedTarget simFuel = new PhotonTrackedTarget(
            45, 30, 70, 20, -1, -1, -1, new Transform3d(5, 6, 0, fuelRotation), new Transform3d(), 0, new LinkedList<>(), new LinkedList<>()
        );
        PhotonTrackedTarget simFuelTwo = new PhotonTrackedTarget(
            45, 30, 100, 20, -1, -1, -1, new Transform3d(1,1,1, fuelRotation), new Transform3d(), 0, new LinkedList<>(), new LinkedList<>()
        );
        
        simTargetList.add(simFuel);
        simTargetList.add(simFuelTwo);
        
        
        return simTargetList;
    }

}