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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.photonvision.simulation.SimCameraProperties;

public class Constants {
    static public final boolean kDebugLoggingEnabled = true;
    static public final double kSimPeriodicUpdateInterval = 0.020;

    static public class VisionK {
        static public final SimCameraProperties kCamera1SimProps = new SimCameraProperties();
        static {
            //TODO: [INSERT CAMERA TYPE]
            kCamera1SimProps.setCalibration(1920, 1080, Rotation2d.fromDegrees(128.2));
            kCamera1SimProps.setCalibError(0.35, 0.10);
            kCamera1SimProps.setFPS(35);
            kCamera1SimProps.setAvgLatencyMs(30);
            kCamera1SimProps.setLatencyStdDevMs(15);
        }

        static public final SimCameraProperties kCamera2SimProps = new SimCameraProperties();
        static {
            //TODO: [INSERT CAMERA TYPE]
            kCamera2SimProps.setCalibration(1280, 720, Rotation2d.fromDegrees(100));
            kCamera2SimProps.setCalibError(0.35, 0.10);
            kCamera2SimProps.setFPS(45);
            kCamera2SimProps.setAvgLatencyMs(25);
            kCamera2SimProps.setLatencyStdDevMs(15);
        }

        static public final String kCamera1CamName = "camera1";
        static public final Transform3d kCamera1CamRoboToCam = new Transform3d(
            //TODO: Update these numbers
            Units.inchesToMeters(8.238), Units.inchesToMeters(4.81), Units.inchesToMeters(32),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(40), Units.degreesToRadians(-10)));
        static public final String kCamera1CamSimVisualName = "camera1VisionEstimation";

        static public final String kCamera2CamName = "camera2";
        static public final Transform3d kCamera2CamRoboToCam = new Transform3d(
            //TODO: Update these numbers
            Units.inchesToMeters(9.964), Units.inchesToMeters(-10.499), Units.inchesToMeters(8.442),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-9.962), Units.degreesToRadians(5)));
        static public final String kCamera2CamSimVisualName = "camera2VisionEstimation";
    }

    static public class FieldK {
        // take with a grain of salt - pulled from field dimensions (welded)
        static public final double kFieldLengthMeters = Units.inchesToMeters(651.22); 
        static public final double kFieldWidthMeters = Units.inchesToMeters(317.69);

        static public final AprilTagFieldLayout kTagLayout;

        //Ignore trench April Tags
        static {
            HashSet<Integer> excludedAprilTagsID = new HashSet<> (Arrays.asList(1, 6, 7, 12, 17, 22, 23, 28));
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            List<AprilTag> tags = new ArrayList<> (fieldLayout.getTags());
            tags.removeIf(tag -> excludedAprilTagsID.contains(tag.ID));
            kTagLayout = new AprilTagFieldLayout(tags, fieldLayout.getFieldLength(), fieldLayout.getFieldWidth());
        }
    }

    static public class RobotK {
        static public final String kLogTab = "Superstructure";
    }

    static public class IntakeK {
        static public final String kLogTab = "Intake";

        /* MOTOR CONSTANTS */
        static public final double kDeployMOI = 0.0209;
        static public final double kDeployGearing = 5/1;

        static public final double kRollersMOI = 0.00005;
        static public final double kRollersGearing = 1.0/2;

        /* IDS */
        static public final int kDeployCANID = 41; //TODO: change to correct
        static public final int kRollersCANID = 42; //TODO: change to correct

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
        static public final MotorOutputConfigs kDeployMotorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
        private static final MotionMagicConfigs kDeployMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(100)
            .withMotionMagicJerk(0);
        static public final FeedbackConfigs kDeployFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1 / kDeployGearing);
        static public final TalonFXConfiguration kDeployConfiguration = new TalonFXConfiguration()
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
        static public final MotorOutputConfigs kRollersMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // TODO: CW or CCW?
            .withNeutralMode(NeutralModeValue.Brake);
        static public final FeedbackConfigs kRollersFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1 / kRollersGearing);
        static public final TalonFXConfiguration kRollersConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kRollersCurrentLimitConfigs)
            .withSlot0(kRollersSlot0Configs)
            .withMotorOutput(kRollersMotorOutputConfigs)
            .withFeedback(kRollersFeedbackConfigs);
    }

    static public class IndexerK {
        static public String kLogTab = "Indexer";
        
        /* IDS */
        //TODO: Make ids accurate
        static public final int kSpinnerCANID = 49;
        static public final int kExhaustCANID = 50;

        static public final double kSpinnerGearing = 3;
        static public final double kExhaustGearing = 1/1.2;

        static public final double kSpinnerMOI = 0.00166190059;
        static public final double kExhaustMOI = 0.000215968064;
        
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
        static public final TalonFXConfiguration kSpinnerTalonFXConfiguration = new TalonFXConfiguration()
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
        static public final TalonFXConfiguration kExhaustTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kExhaustSlot0Configs)
            .withCurrentLimits(kExhaustCurrentLimitConfigs)
            .withMotorOutput(kExhaustMotorOutputConfigs)
            .withFeedback(kExhaustFeedbackConfigs);
    }

    static public class AutonK {
        static public final Pose2d neutralPose = new Pose2d(Distance.ofRelativeUnits(6.924767017364502, Meter), 
            Distance.ofRelativeUnits(2.251265048980713, Meter), new Rotation2d(Math.PI));
        static public final Pose2d depotPose = new Pose2d(Distance.ofRelativeUnits(1.1576627492904663, Meter), 
            Distance.ofRelativeUnits(5.958622932434082, Meter), new Rotation2d(0));
    }
}
