package frc.util;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionUtil {
    public static Transform3d transformToRobo(double x, double y, double z, double rotX, double rotY, double rotZ) {
        x = Units.inchesToMeters(x);
        y = Units.inchesToMeters(y);
        z = Units.inchesToMeters(z);
        rotX = Units.degreesToRadians(rotX);
        rotY = Units.degreesToRadians(rotY);
        rotZ = Units.degreesToRadians(rotZ);
        return new Transform3d(x, y, z, new Rotation3d(rotX, rotY, rotZ));
    }

    public static SimCameraProperties SimCamProps(int resX, int resY, int fovDiag, int fps, double avgErrorPx, double errorStdDevPx, double avgLatencyMs, double latencyStdDevMs) {
        SimCameraProperties simCameraProps = new SimCameraProperties();
        simCameraProps.setCalibration(resX, resY, Rotation2d.fromDegrees(fovDiag));
        simCameraProps.setFPS(fps);
        simCameraProps.setCalibError(avgErrorPx, errorStdDevPx);
        simCameraProps.setAvgLatencyMs(avgLatencyMs);
        simCameraProps.setLatencyStdDevMs(latencyStdDevMs);
        return simCameraProps;
    }

    public static SimCameraProperties SimCamProps(String cameraType, double avgErrorPx, double errorStdDevPx, double avgLatencyMs, double latencyStdDevMs) {
        switch(cameraType) {
            case "ThriftyCam": //https://docs.thethriftybot.com/thriftycam
                return SimCamProps(1600, 1304, 90, 60, avgErrorPx, errorStdDevPx, avgLatencyMs, latencyStdDevMs);
            default:
                return SimCamProps(1600, 1304, 90, 60, avgErrorPx, errorStdDevPx, avgLatencyMs, latencyStdDevMs);
        }
    }
}