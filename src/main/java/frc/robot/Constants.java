package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static final boolean kDebugLoggingEnabled = true;
    public static final double kSimPeriodicUpdateInterval = 0.020;

    public static class ShooterK {
        public static final String kLogTab = "Shooter";
        //cant really think of any other name but is where the turret is relative to the robot
        public static final Transform3d kRobotToTurret = new Transform3d(new Translation3d(Inches.zero(), Inches.zero(), Inches.of(18)), Rotation3d.kZero); //DUMMY VALS
        public static final Distance kDistanceAboveFunnel = Inches.of(20); //distance above the hub funnel

        public static final double kTurretMinAngle = Units.degreesToRadians(-270.0);
        public static final double kTurretMaxAngle = Units.degreesToRadians(270.0);

        public static final double kHoodMinAngle = Units.degreesToRadians(19);
        public static final double kHoodMaxAngle = Units.degreesToRadians(51);

        public static final Distance kFlywheelRadius = Inches.of(2); //fake

        public static final int kHopperCapacity = 55; //TODO: find true max

        //TODO: work out where our passing spots should be..?
        public static final Translation3d kPassingSpotLeft = new Translation3d(
                Inches.of(90), FieldConstants.fieldWidthIn.div(2).plus(Inches.of(85)), Inches.zero());
        public static final Translation3d kPassingSpotCenter = new Translation3d(
                Inches.of(90), FieldConstants.fieldWidthIn.div(2), Inches.zero());

        /* IDS */
        public static final int kLeaderCANID = 21; //TODO: Update CANID number
        public static final int kFollowerCANID = 22; //TODO: Update CANID number
        public static final int kHoodCANID = 23; //TODO: Update CANID number
        public static final int kTurretCANID = 24; //TODO: Update CANID number

        public static final int kExitBeamBreakChannel = 0; //TODO: Update channel number

        /* CONFIGS */
        // TODO: Check what more configs would be necessary
        private static final Slot0Configs kLeaderSlot0Configs = new Slot0Configs()   //Note to self (hrehaan) (and saarth cuz i did the same thing): the default PID sets ZERO volts to a motor, which makes all sim effectively useless cuz the motor has ZERO supplyV
            .withKS(0)
            .withKV(0.1217)
            .withKA(0)
            .withKP(0)
            .withKI(0)
            .withKD(0); // kP was causing the werid sinusoid behavior, kS and kA were adding inconsistency with the destination values
        private static final CurrentLimitsConfigs kLeaderCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(110)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kLeaderOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: check whether this should be CW or CCW
            .withNeutralMode(NeutralModeValue.Brake);
        public static final TalonFXConfiguration kLeaderTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kLeaderSlot0Configs)
            .withCurrentLimits(kLeaderCurrentLimitConfigs)
            .withMotorOutput(kLeaderOutputConfigs);
        
        // TODO: mimics the leader, so it doesn't need its own configs - right?
        public static final TalonFXConfiguration kFollowerTalonFXConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive) //TODO: check whether this should be CW or CCW
                .withNeutralMode(NeutralModeValue.Brake));

        // TODO: I assume we would want the Hood and Turret to move at a constant high velocity
        //       so we should probably configure that here?
        //
        //       that's not exactly how it works - i'm not sure either but kV is more like a boost to velo than velo itself
        //       so in cases like the turret there's no kV
        private static final Slot0Configs kHoodSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(1)
            .withKA(0)
            .withKP(0.85)
            .withKI(0.1)
            .withKD(0); // kP was too low making the slope less steep, kS and kA were adding weird behavior, added kI to account for kP overshooting
        private static final CurrentLimitsConfigs kHoodCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(110)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kHoodOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: check whether this should be CW or CCW
            .withNeutralMode(NeutralModeValue.Brake);
        public static final TalonFXConfiguration kHoodTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kHoodSlot0Configs)
            .withCurrentLimits(kHoodCurrentLimitConfigs)
            .withMotorOutput(kHoodOutputConfigs);
        
        private static final Slot0Configs kTurretSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withKP(3)
            .withKI(0)
            .withKD(0); // kP was too low making the slope less steep, kS kV and kA were causing rlly weird behavior (jumping up/down way further than targeted position)
        private static final CurrentLimitsConfigs kTurretCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(110)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTurretOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: check whether this should be CW or CCW
            .withNeutralMode(NeutralModeValue.Brake)
            .withPeakForwardDutyCycle(0.1)
            .withPeakReverseDutyCycle(0.1);
        private static final MotionMagicConfigs kTurretMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(100)
            .withMotionMagicJerk(0);
        private static final SoftwareLimitSwitchConfigs kTurretSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(0.78)
            .withReverseSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(0.02);
        public static final TalonFXConfiguration kTurretTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kTurretSlot0Configs)
            .withCurrentLimits(kTurretCurrentLimitConfigs)
            .withMotorOutput(kTurretOutputConfigs)
            .withMotionMagic(kTurretMotionMagicConfigs)
            .withSoftwareLimitSwitch(kTurretSoftwareLimitSwitchConfigs);
    }

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
        //these values arent even dummy theyre just stupid
        //TODO: ask hrehaan or build for dimensions
        public static final Distance kRobotFullWidth = Feet.of(6.9);
        public static final Distance kRobotFullLength = Feet.of(6.9);
        public static final Distance kBumperHeight = Feet.of(6.9);

    }
}
