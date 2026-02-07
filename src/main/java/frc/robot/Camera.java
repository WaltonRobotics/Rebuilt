package frc.robot;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Camera {
    private SimCameraProperties m_simCameraProperties;
    private String m_cameraName;
    private String m_simVisualName;
    private Transform3d m_roboToCam;
    
    public Camera(SimCameraProperties simCameraProperties, String cameraName, String simVisualName, Transform3d roboToCam) {
        m_cameraName = cameraName;
        m_simVisualName = cameraName + simVisualName;
        m_roboToCam = roboToCam;
        m_simCameraProperties = simCameraProperties;
    }

    /**
     * Helper function to convert from user-friendly units (inches and degrees) to the units used by the simulation (meters and radians).
     * @param x in inches
     * @param y in inches
     * @param z in inches
     * @param rotX in degrees
     * @param rotY in degrees
     * @param rotZ in degrees
     * @return a Transform3d with the converted values
     */
    public static Transform3d transformToRobo(double x, double y, double z, double rotX, double rotY, double rotZ) {
        x = Units.inchesToMeters(x);
        y = Units.inchesToMeters(y);
        z = Units.inchesToMeters(z);
        rotX = Units.degreesToRadians(rotX);
        rotY = Units.degreesToRadians(rotY);
        rotZ = Units.degreesToRadians(rotZ);
        return new Transform3d(x, y, z, new Rotation3d(rotX, rotY, rotZ));
    }

    public static Transform3d transformToRobo(double[] posAndRot) {
        return transformToRobo(posAndRot[0], posAndRot[1], posAndRot[2], posAndRot[3], posAndRot[4], posAndRot[5]);
    }

    public void setRoboToCam(Transform3d roboToCam) {
        m_roboToCam = roboToCam;
    }

    /**
     * Sets the camera's simulated properties.
     * @param resX Resolution in the X direction (pixels)
     * @param resY Resolution in the Y direction (pixels)
     * @param fovDiag Diagonal field of view (degrees)
     */
    public void setCalibration(int resX, int resY, double fovDiag) {
        m_simCameraProperties.setCalibration(resX, resY, Rotation2d.fromDegrees(fovDiag));
    };

    public void setCalibError(double avgErrorPx, double errorStdDevPx) {
        m_simCameraProperties.setCalibError(avgErrorPx, errorStdDevPx);
    };

    public void setFPS(int fps) {
        m_simCameraProperties.setFPS(fps);
    };

    public void setProps(int resX, int resY, double fovDiag, double avgErrorPx, double errorStdDevPx, int fps, int avgLatencyMs, int latencyStdDevMs) {
        setCalibration(resX, resY, fovDiag);
        setCalibError(avgErrorPx, errorStdDevPx);
        setFPS(fps);
        setLatency(avgLatencyMs, latencyStdDevMs);
    }

    public void setLatency(int avgLatencyMs, int latencyStdDevMs) {
        m_simCameraProperties.setAvgLatencyMs(avgLatencyMs);
        m_simCameraProperties.setLatencyStdDevMs(latencyStdDevMs);
    }

    public Transform3d getRoboToCam() {
        return m_roboToCam;
    }

    public String getCameraName() {
        return m_cameraName;
    }

    public String getSimVisualName() {
        return m_simVisualName;
    }

    public SimCameraProperties getSimCameraProperties() {
        return m_simCameraProperties;
    }
}