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

    public void setRoboToCam(Transform3d roboToCam) {
        m_roboToCam = roboToCam;
    }

    /**
     * Sets the camera's simulated properties.
     * @param resX Resolution in the X direction (pixels)
     * @param resY Resolution in the Y direction (pixels)
     * @param fovDiag Diagonal field of view (degrees)
     * @param fps Frames per second
     * @param avgErrorPx Average calibration error (pixels)
     * @param errorStdDevPx Standard deviation of calibration error (pixels)
     * @param avgLatencyMs Average latency (milliseconds)
     * @param latencyStdDevMs Standard deviation of latency (milliseconds)
     */
    public void setProps(int resX, int resY, double fovDiag, int fps, double avgErrorPx, double errorStdDevPx, double avgLatencyMs, double latencyStdDevMs) {
        setCameraSpecs(resX, resY, fovDiag, fps);
        setCalibError(avgErrorPx, errorStdDevPx);
        setLatency(avgLatencyMs, latencyStdDevMs);
    }

    public void setProps(String cameraType, double avgErrorPx, double errorStdDevPx, double avgLatencyMs, double latencyStdDevMs) {
        setCameraSpecs(cameraType);
        setCalibError(avgErrorPx, errorStdDevPx);
        setLatency(avgLatencyMs, latencyStdDevMs);
    }

    /**
     * @param resX Resolution in the X direction (pixels)
     * @param resY Resolution in the Y direction (pixels)
     * @param fovDiag Diagonal field of view (degrees)
     * @param fps Frames per second
     */
    public void setCameraSpecs(int resX, int resY, double fovDiag, int fps) {
        m_simCameraProperties.setCalibration(resX, resY, Rotation2d.fromDegrees(fovDiag));
        m_simCameraProperties.setFPS(fps);
    }

    /**
     * Sets the camera's simulated error properties.
     * @param avgErrorPx Average calibration error (pixels)
     * @param errorStdDevPx Standard deviation of calibration error (pixels)
     */
    public void setCalibError(double avgErrorPx, double errorStdDevPx) {
        m_simCameraProperties.setCalibError(avgErrorPx, errorStdDevPx);
    }

    /**
     * Sets the camera's simulated latency properties.
     * @param avgLatencyMs Average latency (milliseconds)
     * @param latencyStdDevMs Standard deviation of latency (milliseconds)
     */
    public void setLatency(double avgLatencyMs, double latencyStdDevMs) {
        m_simCameraProperties.setAvgLatencyMs(avgLatencyMs);
        m_simCameraProperties.setLatencyStdDevMs(latencyStdDevMs);
    }

    /**
     * Sets the camera's simulated properties based on a predefined camera type.
     * does nothing if the camera type is not recognized
     * @param cameraType ex: "ThriftyCam"
     */
    public void setCameraSpecs(String cameraType) {
        switch(cameraType) {
            case "ThriftyCam": //https://docs.thethriftybot.com/thriftycam
                setCameraSpecs(1600, 1304, 55, 60);
                break;
            default:
                break;
        }
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