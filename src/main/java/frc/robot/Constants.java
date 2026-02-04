package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

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

        public static final AprilTagFieldLayout kTagLayout;

        //Ignore trench April Tags
        static {
            HashSet<Integer> excludedAprilTagsID = new HashSet<> (Arrays.asList(1, 6, 7, 12, 17, 22, 23, 28));
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            List<AprilTag> tags = new ArrayList<> (fieldLayout.getTags());
            tags.removeIf(tag -> excludedAprilTagsID.contains(tag.ID));
            kTagLayout = new AprilTagFieldLayout(tags, fieldLayout.getFieldLength(), fieldLayout.getFieldWidth());
        }
    }

    public static class AutoAlignK {
        //Placeholder tolerances - to be tuned
        //Delete this comment when done tuning
        public static final Distance kFieldTranslationTol = Meters.of(0); //meters
        public static final Angle kFieldRotationTol = Degrees.of(0); //degrees

        public static final double kFinishVelTol = 0; //meters per second

        public static final double kIntermediatePoseDistance = -Units.inchesToMeters(6); // value in meters
        public static final Transform2d kIntermediatePoseTransform 
            = new Transform2d(kIntermediatePoseDistance, 0, Rotation2d.kZero);

            
        public static double kXKP = 8;
        public static double kYKP = 8;
        public static double kThetaKP = 10;

        // SUPER COOL AUTO ALIGN :sunglasses: - this should eventually allow you to replace all code using above constants

        // TODO: these will really need tuning
        // teleop speeds below
        public static final double kMaxDimensionVel = 1.65; // m/s
        public static final double kMaxDimensionAccel = 6; // m/s^2
        public static final TrapezoidProfile.Constraints kXYConstraints = new TrapezoidProfile.Constraints(kMaxDimensionVel, kMaxDimensionAccel);
        // Auton speeds below
        public static final double kMaxDimensionVelEleUp = 2; // m/s
        public static final double kMaxDimensionAccelEleUp = 3; // m/s^2
        public static final TrapezoidProfile.Constraints kXYConstraintsAuton 
            = new TrapezoidProfile.Constraints(kMaxDimensionVelEleUp,kMaxDimensionAccelEleUp);

        public static final double kMaxThetaVel = 4; // rad/s
        public static final double kMaxThetaAccel = 8; // rad/s^2
        public static final TrapezoidProfile.Constraints kThetaConstraints = new TrapezoidProfile.Constraints(kMaxThetaVel, kMaxThetaAccel);
        
        /** <p>Arbitrary number to control how much a difference in rotation should affect tag selection. Higher means more weight
         * <p> 0 means rotation difference has no weight, negative will literally bias it against tags that have more similar rotations */
        public static final double kRotationWeight = 0.2;

        public static final double kFutureDelta = 0.3; // seconds, TODO: needs tuning
    }

    public static class RobotK {
        public static final String kLogTab = "Superstructure";
    }
}
