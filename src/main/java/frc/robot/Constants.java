package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final boolean kDebugLoggingEnabled = true;
    public static final double kSimPeriodicUpdateInterval = 0.020;

    public static class VisionK {
        public static final Camera[] kCameras = new Camera[4];
        private static final String[] kCameraNames = {
            "frontLeftCamera",
            "frontRightCamera",
            "backLeftCamera",
            "backRightCamera"
        };
        private static final String kSimCameraSimVisualNames = "VisionEstimation"; //suffixed to each camera name
        //TODO: Make these transforms accurate - they are currently just placeholders
        private static final Transform3d[] kSimCameraRoboToCam = {
            Camera.transformToRobo(0 , 0, 0, 0, 0, 0 ),
            Camera.transformToRobo(0 , 0, 0, 0, 0, 0 ),
            Camera.transformToRobo(0 , 0, 0, 0, 0, 0 ),
            Camera.transformToRobo(0 , 0, 0, 0, 0, 0 )
        };
        //TODO: Make these values accurate - they are currently just placeholders
        private static final double[] kSimCamAvgError = {0, 0, 0, 0};
        private static final double[] kSimCamErrorStdDev = {0, 0, 0, 0};
        private static final int[] kSimCamAvgLatencyMs = {0, 0, 0, 0};
        private static final int[] kSimCamLatencyStdDevMs = {0, 0, 0, 0};

        //Initialize cameras
        static {
            for(int i = 0; i < kCameras.length; i++) {
                kCameras[i] = new Camera(new SimCameraProperties(), kCameraNames[i], kSimCameraSimVisualNames, kSimCameraRoboToCam[i]);
                kCameras[i].setCameraSpecs("ThriftyCam");
                kCameras[i].setCalibError(kSimCamAvgError[i], kSimCamErrorStdDev[i]);
                kCameras[i].setLatency(kSimCamAvgLatencyMs[i], kSimCamLatencyStdDevMs[i]);
            }
        }
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

    public static class RobotK {
        public static final String kLogTab = "Superstructure";
    }

    public static class IndexerK {
        public static String kLogTab = "Indexer";
        
        /* IDS */
        //TODO: Make ids accurate
        public static final int kSpinnerCANID = 49;
        public static final int kExhaustCANID = 50;

        public static final double kSpinnerGearing = 3;
        public static final double kExhaustGearing = 1/1.2;

        public static final double kSpinnerMOI = 0.00166190059;
        public static final double kExhaustMOI = 0.000215968064;
        
        /* CONFIGS */
        //TODO: Make transfer configs accurate
        private static final Slot0Configs kSpinnerSlot0Configs = new Slot0Configs()
            .withKS(0.012)
            .withKV(0.371)
            .withKA(0)
            .withKP(0.1)
            .withKI(0)
            .withKD(0.03);
        private static final CurrentLimitsConfigs kSpinnerCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(70)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kSpinnerMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: CW or CCW?
            .withNeutralMode(NeutralModeValue.Brake);
        private static final FeedbackConfigs kSpinnerFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kSpinnerGearing);
        public static final TalonFXConfiguration kSpinnerTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kSpinnerSlot0Configs)
            .withCurrentLimits(kSpinnerCurrentLimitConfigs)
            .withMotorOutput(kSpinnerMotorOutputConfigs)
            .withFeedback(kSpinnerFeedbackConfigs);

        private static final Slot0Configs kExhaustSlot0Configs = new Slot0Configs()
            .withKS(0.1124)
            .withKV(0.102)
            .withKA(0)
            .withKP(0.06)
            .withKI(0)
            .withKD(0);
        private static final CurrentLimitsConfigs kExhaustCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(140)
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kExhaustMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: CW or CCW?
            .withNeutralMode(NeutralModeValue.Brake);
        private static final FeedbackConfigs kExhaustFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kExhaustGearing);
        public static final TalonFXConfiguration kExhaustTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kExhaustSlot0Configs)
            .withCurrentLimits(kExhaustCurrentLimitConfigs)
            .withMotorOutput(kExhaustMotorOutputConfigs)
            .withFeedback(kExhaustFeedbackConfigs);
    }
}
