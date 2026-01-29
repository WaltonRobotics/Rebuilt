package frc.robot.vision;

import static frc.robot.Constants.FieldK.kTagLayout;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    public Command addFuel() {
        TargetModel targetModel = new TargetModel(Units.inchesToMeters(5.91));
        Pose3d targetPose = new Pose3d(0,0,0, new Rotation3d(0,0,Math.PI)); //dummy values
        VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

        return Commands.sequence(
            Commands.runOnce(() -> m_photonVisionSim.addVisionTargets(visionTarget)),
            Commands.print("--------------------ADDED FUEL--------------------")
        );
    }
}