package frc.robot;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final boolean kDebugLoggingEnabled = true;

    public static class VisionK {
        public static final SimCameraProperties kCamera1SimProps = new SimCameraProperties();
        static {
            //TODO: [INSERT CAMERA TYPE]
            kCamera1SimProps.setCalibration(1920, 1080, Rotation2d.fromDegrees(128.2));
            kCamera1SimProps.setCalibError(0.35, 0.10);
            kCamera1SimProps.setFPS(35);
            kCamera1SimProps.setAvgLatencyMs(30);
            kCamera1SimProps.setLatencyStdDevMs(15);
        }

        public static final SimCameraProperties kCamera2SimProps = new SimCameraProperties();
        static {
            //TODO: [INSERT CAMERA TYPE]
            kCamera2SimProps.setCalibration(1280, 720, Rotation2d.fromDegrees(100));
            kCamera2SimProps.setCalibError(0.35, 0.10);
            kCamera2SimProps.setFPS(45);
            kCamera2SimProps.setAvgLatencyMs(25);
            kCamera2SimProps.setLatencyStdDevMs(15);
        }
        
        public static final String kCamera1CamName = "camera1";
        public static final Transform3d kCamera1CamRoboToCam = new Transform3d(
            //TODO: Update these numbers
            Units.inchesToMeters(8.238), Units.inchesToMeters(4.81), Units.inchesToMeters(32), 
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(40), Units.degreesToRadians(-10))
        );
        public static final String kCamera1CamSimVisualName = "camera1VisionEstimation";

        public static final String kCamera2CamName = "camera2";
        public static final Transform3d kCamera2CamRoboToCam = new Transform3d(
            //TODO: Update these numbers
            Units.inchesToMeters(9.964), Units.inchesToMeters(-10.499), Units.inchesToMeters(8.442),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-9.962), Units.degreesToRadians(5))
        );
        public static final String kCamera2CamSimVisualName = "camera2VisionEstimation";
    }

    public static class FieldK {
        public static final double kFieldLengthMeters = Units.inchesToMeters(651.22); //take with a grain of salt - pulled from field dimensions (welded)
        public static final double kFieldWidthMeters = Units.inchesToMeters(317.69);

        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }

    public static class RobotK {
        public static final String kLogTab = "Superstructure";
    }
}
