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

        public static final int kIntakeDeployCANID = 41;
        public static final int kIntakeRollerCANID = 42; // TODO CHANGE TO CORRECT

        // deploy motor
        public static final int kDeployGearRatio = 40; // TODO: check if still accurate
        public static final int kDeploySensorToMechanismRatio = kDeployGearRatio;
        public static final int kAngleTolerance = 3; // DUMMY VALUE
        public static final double kDeployMMVelo = 1;
        public static final double kDeployMMAccel = 25;
        public static final double kDeployMMJerk = 300;
        public static final double kDeployMMVeloSlow = 0.65;
        public static final double kDeployMMAccelSlow = 10;

        private static final CurrentLimitsConfigs kDeployCurrentLimitConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(20)
                .withSupplyCurrentLimit(20)
                .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kDeployFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(kDeploySensorToMechanismRatio);
        private static final MotionMagicConfigs kDeployMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(kDeployMMVelo))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(kDeployMMAccel))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(kDeployMMJerk));
        private static final Slot0Configs kDeploySlot0Configs = new Slot0Configs()
                .withKS(0)
                .withKV(10)
                .withKA(0)
                .withKP(20)
                .withKI(0.1)
                .withKD(0);
        public static final MotorOutputConfigs kDeployMotorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
        private static final MotionMagicConfigs kDeployMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(100)
            .withMotionMagicJerk(0);
        public static final TalonFXConfiguration kDeployConfiguration = new TalonFXConfiguration()
                .withCurrentLimits(kDeployCurrentLimitConfigs)
                .withFeedback(kDeployFeedbackConfigs)
                .withMotionMagic(kDeployMagicConfigs)
                .withSlot0(kDeploySlot0Configs)
                .withMotorOutput(kDeployMotorOutputConfigs)
                .withMotionMagic(kDeployMotionMagicConfigs);

        // intake motor
        public static final int kRollerGearRatio = 2;
        public static final int kRollerSensorToMechanismRatio = kRollerGearRatio;
        private static final CurrentLimitsConfigs kRollerCurrentLimitConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(110)
                .withSupplyCurrentLimit(50)
                .withStatorCurrentLimitEnable(true);
        private static final FeedbackConfigs kRollerFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(kRollerSensorToMechanismRatio);
        private static final MotionMagicConfigs kRollerMotionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(10));
        private static final Slot0Configs kRollerSlot0Configs = new Slot0Configs()
                .withKS(1)
                .withKV(1)
                .withKA(1)
                .withKP(1)
                .withKI(1)
                .withKD(1);
        public static final MotorOutputConfigs kRollerMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);
        public static final TalonFXConfiguration kRollerConfiguration = new TalonFXConfiguration()
                .withCurrentLimits(kRollerCurrentLimitConfigs)
                .withFeedback(kRollerFeedbackConfigs)
                .withSlot0(kRollerSlot0Configs)
                .withMotorOutput(kRollerMotorOutputConfigs)
                .withMotionMagic(kRollerMotionMagicConfigs);
    }
}