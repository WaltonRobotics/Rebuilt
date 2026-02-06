package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.photonvision.simulation.SimCameraProperties;

public class Constants {
    public static final boolean kDebugLoggingEnabled = true;

    public static class VisionK {
        public static final SimCameraProperties kCamera1SimProps = new SimCameraProperties();
        static {
            // TODO: [INSERT CAMERA TYPE]
            kCamera1SimProps.setCalibration(1920, 1080, Rotation2d.fromDegrees(128.2));
            kCamera1SimProps.setCalibError(0.35, 0.10);
            kCamera1SimProps.setFPS(35);
            kCamera1SimProps.setAvgLatencyMs(30);
            kCamera1SimProps.setLatencyStdDevMs(15);
        }

        public static final SimCameraProperties kCamera2SimProps = new SimCameraProperties();
        static {
            // TODO: [INSERT CAMERA TYPE]
            kCamera2SimProps.setCalibration(1280, 720, Rotation2d.fromDegrees(100));
            kCamera2SimProps.setCalibError(0.35, 0.10);
            kCamera2SimProps.setFPS(45);
            kCamera2SimProps.setAvgLatencyMs(25);
            kCamera2SimProps.setLatencyStdDevMs(15);
        }

        public static final String kCamera1CamName = "camera1";
        public static final Transform3d kCamera1CamRoboToCam = new Transform3d(
                // TODO: Update these numbers
                Units.inchesToMeters(8.238), Units.inchesToMeters(4.81), Units.inchesToMeters(32),
                new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(40), Units.degreesToRadians(-10)));
        public static final String kCamera1CamSimVisualName = "camera1VisionEstimation";

        public static final String kCamera2CamName = "camera2";
        public static final Transform3d kCamera2CamRoboToCam = new Transform3d(
                // TODO: Update these numbers
                Units.inchesToMeters(9.964), Units.inchesToMeters(-10.499), Units.inchesToMeters(8.442),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-9.962), Units.degreesToRadians(5)));
        public static final String kCamera2CamSimVisualName = "camera2VisionEstimation";
    }

    public static class FieldK {
        public static final double kFieldLengthMeters = Units.inchesToMeters(651.22); // take with a grain of salt -
                                                                                      // pulled from field dimensions
                                                                                      // (welded)
        public static final double kFieldWidthMeters = Units.inchesToMeters(317.69);

        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);
    }

    public static class RobotK {
        public static final String kLogTab = "Superstructure";
    }

    public static class IntakeK {
        public static final String kLogTab = "Intake";

        /* MOTOR CONSTANTS */

        // deploy motor
        public static final double kDeployMomentOfInertia = 0.0209;
        public static final double kDeployGearing = 5/1;

        public static final int kDeployCANID = 41; // TODO change to correct

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

        // rollers motor
        public static final double kRollersMomentOfInertia = 0.00005;
        public static final double kRollersGearing = 1.0/2;

        public static final int kRollersCANID = 42; // TODO change to correct

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
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        public static final FeedbackConfigs kRollersFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(1 / kRollersGearing);
        public static final TalonFXConfiguration kRollersConfiguration = new TalonFXConfiguration()
                .withCurrentLimits(kRollersCurrentLimitConfigs)
                .withSlot0(kRollersSlot0Configs)
                .withMotorOutput(kRollersMotorOutputConfigs)
                .withFeedback(kRollersFeedbackConfigs);
    }
}