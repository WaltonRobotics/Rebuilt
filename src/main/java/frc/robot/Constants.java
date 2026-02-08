package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.photonvision.simulation.SimCameraProperties;

public class Constants {

    public static final boolean kDebugLoggingEnabled = true;
    public static final double kSimPeriodicUpdateInterval = 0.020;

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
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(40), Units.degreesToRadians(-10)));
        public static final String kCamera1CamSimVisualName = "camera1VisionEstimation";

        public static final String kCamera2CamName = "camera2";
        public static final Transform3d kCamera2CamRoboToCam = new Transform3d(
            //TODO: Update these numbers
            Units.inchesToMeters(9.964), Units.inchesToMeters(-10.499), Units.inchesToMeters(8.442),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-9.962), Units.degreesToRadians(5)));
        public static final String kCamera2CamSimVisualName = "camera2VisionEstimation";
    }

    public static class FieldK {
        // take with a grain of salt - pulled from field dimensions (welded)
        public static final double kFieldLengthMeters = Units.inchesToMeters(651.22); 
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

    public static class IntakeK {
        public static final String kLogTab = "Intake";

        /* MOTOR CONSTANTS */
        public static final double kDeployMOI = 0.0209;
        public static final double kDeployGearing = 5/1;

        public static final double kRollersMOI = 0.00005;
        public static final double kRollersGearing = 1.0/2;

        /* IDS */
        public static final int kDeployCANID = 41; //TODO: change to correct
        public static final int kRollersCANID = 42; //TODO: change to correct

        /* CONFIGS */
        //Deploy Motor
        private static final CurrentLimitsConfigs kDeployCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true);
        private static final Slot0Configs kDeploySlot0Configs = new Slot0Configs()
            .withKS(0.04)
            .withKV(0)
            .withKA(0)
            .withKP(0.8)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kDeployMotorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
        private static final MotionMagicConfigs kDeployMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(100)
            .withMotionMagicJerk(0);
        public static final FeedbackConfigs kDeployFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1 / kDeployGearing);
        public static final TalonFXConfiguration kDeployConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kDeployCurrentLimitConfigs)
            .withSlot0(kDeploySlot0Configs)
            .withMotorOutput(kDeployMotorOutputConfigs)
            .withMotionMagic(kDeployMotionMagicConfigs);

        //Rollers Motor
        private static final CurrentLimitsConfigs kRollersCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final Slot0Configs kRollersSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(0.19543973941)
            .withKA(0)
            .withKP(0)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kRollersMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // TODO: CW or CCW?
            .withNeutralMode(NeutralModeValue.Brake);
        public static final FeedbackConfigs kRollersFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1 / kRollersGearing);
        public static final TalonFXConfiguration kRollersConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kRollersCurrentLimitConfigs)
            .withSlot0(kRollersSlot0Configs)
            .withMotorOutput(kRollersMotorOutputConfigs)
            .withFeedback(kRollersFeedbackConfigs);
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

    public static final robotMode kSimMode = robotMode.REPLAY;
    public static final robotMode currentMode = RobotBase.isReal() ? robotMode.REAL : kSimMode;

    public enum robotMode {
        REAL,
        SIM,
        REPLAY
    }
}
