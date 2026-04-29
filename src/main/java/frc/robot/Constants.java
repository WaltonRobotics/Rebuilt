
package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.IndexerK.kSpindexerShootRPSD;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.util.AllianceFlipUtil;
import frc.util.VisionUtil;

public class Constants {
    public static final boolean kDebugLoggingEnabled = false;
    public static final boolean kDataLoggingEnabled = true;
    public static final double kSimPeriodicUpdateInterval = 0.020;

    public static final CANBus kRioBus = CANBus.roboRIO();
    public static final CANBus kCanivoreBus = new CANBus("fd");
    public static final CANBus kShooterBus = new CANBus("shooter");

    public static final class MotorK {
        public static final double kX60MaxRadPerSec = DCMotor.getKrakenX60(1).freeSpeedRadPerSec;
        public static final AngularVelocity kX60MaxVelocity = RadiansPerSecond.of(kX60MaxRadPerSec);
        public static final double kX60FOCMaxRadPerSec = DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec;
        public static final AngularVelocity kX60FOCMaxVelocity = RadiansPerSecond.of(kX60MaxRadPerSec);

        public static final double kX44MaxRadPerSec = DCMotor.getKrakenX44(1).freeSpeedRadPerSec;
        public static final AngularVelocity kX44MaxVelocity = RadiansPerSecond.of(kX44MaxRadPerSec);
        public static final double kX44FOCMaxRadPerSec = DCMotor.getKrakenX44Foc(1).freeSpeedRadPerSec;
        public static final AngularVelocity kX44FOCMaxVelocity = RadiansPerSecond.of(kX44MaxRadPerSec);
    }
    public static class WpiK {
        public static final ChassisSpeeds kZeroChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }
    public static class ShooterK {
        public static final String kLogTab = "Shooter";
        public static final Rotation2d kTurretAngleOffset = Rotation2d.fromRotations(0.106 + 0.0067); //4.87        //0.132324  // was 0.12, decreased 0.014 (~5deg) to fix consistent rightward aim error
        public static final Rotation3d kTurretAngleOffset3d = new Rotation3d(kTurretAngleOffset);
        public static final Translation3d kTurretTranslation = new Translation3d(Inches.of(-4.744), Inches.of(-4.239), Inches.of(15.769));
        public static final Transform3d kTurretTransformNoRotation = new Transform3d(kTurretTranslation, Rotation3d.kZero);
        public static final Transform3d kTurretTransform = new Transform3d(kTurretTranslation, kTurretAngleOffset3d);
        public static final Distance kInchesAboveFunnel = Inches.of(20);// distance the ball must travel above the funnel opening to arc correctly into the hub
        public static final Angle kTurretBarfPos = Rotations.of(-0.113);

        public static final boolean kUseStaticShot = false;
        public static final boolean kAllowDriverRPSTweak = false;

        // private static final Pose3dLogger log_turretTransform = WaltLogger.logPose3d(kLogTab, "TurretTransformRaw");
        // static {
        //     log_turretTransform.accept(kTurretTransform);
        // }

        public static final Distance kFlywheelRadius = Inches.of(1.5);

        // Precomputed doubles for hot-path shot calc (avoid measure allocations)
        public static final double kTurretOffsetX_m = kTurretTransform.getTranslation().getX();
        public static final double kTurretOffsetY_m = kTurretTransform.getTranslation().getY();
        public static final double kTurretAngleOffsetRad = kTurretAngleOffset.getRadians();
        public static final double kFlywheelRadiusM = kFlywheelRadius.in(Meters);
        public static final double kFlywheelRadiusIn = kFlywheelRadius.in(Inches);
        public static final double kTurretOffsetZ_in = kTurretTransform.getTranslation().getMeasureZ().in(Inches);
        public static final double kFunnelRadiusIn = FieldConstants.Hub.funnelRadius.in(Inches);
        public static final double kFunnelHeightPlusAboveIn = FieldConstants.Hub.funnelHeight.plus(kInchesAboveFunnel).in(Inches);

        // Lateral bias compensation: balls drift left/right as a function of turret angle relative to robot.
        // sin(turretRelAngle) = 0 at 0/180°, +1 at 90° (left bias), -1 at 270° (right bias).
        // This gain (in rotations) is subtracted * sin to counter the bias. Tune on robot.
        public static final double kTurretLateralBiasGainRots = 0;//-0.005

        public static final int kHopperCapacity = 55; //TODO: find true max

        public static final double kGravity = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);

        //TODO: work out where our passing spots should be..?
        // public static final Translation3d kPassingSpotRight = new Translation3d(
        //     Meters.of(2), Meters.of(1.5), Meters.zero());
        // public static final Translation3d kPassingSpotLeft = new Translation3d(
        //     Meters.of(2), Meters.of(6.5), Meters.zero());
        public static final Distance kPassingX = Meters.of(3.5);
        public static final double kPassingXAsDouble = kPassingX.in(Meters);

        public static final double kNoPassZoneTopX = FieldConstants.Hub.blueInnerCenterPoint.getX() + 2;
        public static final double kNoPassZoneRightY = Meters.of(3.2).baseUnitMagnitude();
        public static final double kNoPassZoneLeftY = FieldConstants.fieldWidth - 3.2;
        
        public static final double kShooterTimeout = 1.0;
        public static final double kBallDetectedDebounceTime = 1.2; //0.9;

        /* MOTOR CONSTANTS */
        public static final double kShooterMoI = 0.000349 * 2.5;  //J for 5 3" 0.53lb flywheels
        public static final double kTurretMoI = 0.104506595;

        public static final double kShooterGearing = 1/1;
        public static final double kTurretGearing = 41.66666666/1;
        public static final double kHoodGearing = 25.0/1;

        public static final int kPeakShooterVolts = 16;

        public static final Angle kTurretMaxRotsFromHome = Rotations.of(0.55); //0.75 rots in each direction from home
        public static final Angle kTurretMinRots = Rotations.of(-kTurretMaxRotsFromHome.in(Rotations));
        public static final Angle kTurretMaxRots = Rotations.of(kTurretMaxRotsFromHome.in(Rotations));
        public static final double kTurretMaxErrD = Rotations.of(0.05).in(Rotations);
        public static final double kTurretMaxErrDSpin = Rotations.of(0.4).in(Rotations);

        public static final double kTurretMaxNotAbleToPassRange = 0.23; //sorry for this horrible name i dont know a better one lol
        public static final double kTurretMinNotAbleToPassRange = -0.23; //sorry for this horrible name i dont know a better one lol

        public static final AngularVelocity kShooterMaxRPS = MotorK.kX44MaxVelocity.div(kShooterGearing);
        public static final double kShooterMaxRPSd = kShooterMaxRPS.in(RotationsPerSecond);
        public static final AngularVelocity kShooterRPS = kShooterMaxRPS.times(0.65);   //Kraken X44 Max RPM: 7758
        public static final double kShooterRPSd = 42.90 + 1.25;
        public static final AngularVelocity kShooterAutonCloseRPS = kShooterMaxRPS.times(0.60);  //auton pose is closer to the hub than teleop scoring
        public static final AngularVelocity kShooterAuton_EndSweep_RPS = kShooterMaxRPS.times(0.70); // end of sweep paths
        public static final AngularVelocity kShooterBarfRPS = kShooterMaxRPS.times(0.37);
        public static final AngularVelocity kShooterZeroRPS = RotationsPerSecond.zero();

        public static final AngularVelocity kShooterSpunUpMinimum = RotationsPerSecond.of(10);
        public static final Time kShooterSpunUpTimeout = Seconds.of(0.64);  //double expected spinup time
        public static final double kShooterSpunUpMinimumD = kShooterSpunUpMinimum.in(RotationsPerSecond);

        public static final double kDriverRPSIncreaseD = 2.0;  //biggest cope of the century

        //---HOOD CONSTANTS
        public static final double kHoodMoI = 0.00027505;

        public static final Angle kHoodAbsoluteMinRots = Rotations.of(0.0); //ABSOLUTE MIN
        private static final Angle kHoodAbsoluteMaxRots = Rotations.of(1.174805); //ABSOLUTE MAX
        public static final Angle kHoodMaxDegs = Degrees.of(kHoodAbsoluteMaxRots.in(Degrees));
        public static final Angle kHoodLockDegs = Degrees.of(kHoodMaxDegs.times(0.75).in(Degrees));
        public static final double kHoodRotsd = 0.08;
        public static final double kHoodRotsHalfwayD = kHoodAbsoluteMaxRots.magnitude() * 0.75;
        public static final double kHoodEmergencyRotsD = Rotations.of(0.371338).magnitude();
        public static final double kHoodMaxErrD = Rotations.of(0.01).in(Rotations);



        //double versions
        public static final double kHoodMinRots_double = 0.0;
        public static final double kHoodMaxRots_double = kHoodAbsoluteMaxRots.in(Rotations);
        public static final double kPhysicalHoodMinPosition_double = 0;
        public static final double kPhysicalHoodMaxPosition_double = 48;

        //TODO: ensure this is the home value
        // public static final Angle kHoodHomePosition = Degrees.of(10);
        public static final Angle kHoodTrenchPosition = Degrees.of(5);

        public static final DCMotor khoodDCMotorGearbox = new DCMotor(
            6, 
            0.047, 
            2.5, 
            0.2, 
            24.0855, 
            1
        );

        /* HOMING */
        public static final Current kWireTugMinAmps = Amps.of(8);
        public static final double kWireTugMinSecs = 0.125;
        public static final double kHoodHomingVoltage = -0.75;
        public static final Angle kHomingRetryReturnRots = Rotations.of(0.2);
        public static final Angle kHomePosition = Rotations.of(-0.2175);
        public static final Angle kInitPosition = Rotations.of(-0.145);

        /* IDS */
        public static final int kShooterA_CANID = 21;
        public static final int kShooterB_CANID = 20;
        public static final int kTurretCANID = 12;
        public static final int kHoodCANID = 22;

        /* CONFIGS */
        // TODO: Check what more configs would be necessary
        private static final Slot0Configs kShooterASlot0Configs = new Slot0Configs()   //Note to self (hrehaan) (and saarth cuz i did the same thing): the default PID sets ZERO volts to a motor, which makes all sim effectively useless cuz the motor has ZERO supplyV
            .withKS(0.37)
            .withKV(0.1)
            .withKA(0)
            .withKP(0.5)
            .withKI(0)
            .withKD(0); // kP was causing the werid sinusoid behavior, kS and kA were adding inconsistency with the destination values
        private static final CurrentLimitsConfigs kShooterACurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kShooterAOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kShooterAFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kShooterGearing);
        private static final VoltageConfigs kShooterAVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(kPeakShooterVolts)
            .withPeakReverseVoltage(-kPeakShooterVolts);
        private static final Slot1Configs kShooterASlot1Configs = new Slot1Configs()
            .withKP(5)
            .withKI(0)
            .withKD(0)
            .withKS(4.5)
            .withKV(0.16)
            .withKA(0);
        public static final TalonFXConfiguration kShooterATalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kShooterASlot0Configs)
            .withSlot1(kShooterASlot1Configs)
            .withCurrentLimits(kShooterACurrentLimitConfigs)
            .withMotorOutput(kShooterAOutputConfigs)
            .withFeedback(kShooterAFeedbackConfigs)
            .withVoltage(kShooterAVoltageConfigs);



        private static final MotorOutputConfigs kShooterBOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        public static final TalonFXConfiguration kShooterBTalonFXConfiguration = kShooterATalonFXConfiguration.clone()
            .withMotorOutput(kShooterBOutputConfigs);

        //---HOOD
        private static final Slot0Configs kHoodSlot0Configs = new Slot0Configs()
            .withKP(29)
            .withKI(0)
            .withKD(0)
            .withKS(0.5)
            .withKV(4)
            .withKA(0)
            .withKG(0);
        private static final CurrentLimitsConfigs kHoodCurrentLimitConfig = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(30)
            .withSupplyCurrentLimit(15)
            .withSupplyCurrentLowerLimit(5)
            .withSupplyCurrentLowerTime(1)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kHoodOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        private static final VoltageConfigs kHoodVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(16)
            .withPeakReverseVoltage(-16);
        private static final CommutationConfigs kHoodCommutationConfigs = new CommutationConfigs()
            .withAdvancedHallSupport(AdvancedHallSupportValue.Enabled)
            .withMotorArrangement(MotorArrangementValue.NEO550_JST);
        private static final ExternalFeedbackConfigs kHoodFeedbackConfigs = new ExternalFeedbackConfigs()
            .withSensorToMechanismRatio(kHoodGearing);
        public static final SoftwareLimitSwitchConfigs kHoodSoftLimitConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(kHoodAbsoluteMaxRots.minus(Rotations.of(0.05)))
            .withReverseSoftLimitThreshold(kHoodAbsoluteMinRots.plus(Rotations.of(0.05)))
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true);
        public static final SoftwareLimitSwitchConfigs kHoodSoftLimitConfigsNoEnable = kHoodSoftLimitConfigs
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false); 
        public static final TalonFXSConfiguration kHoodTalonFXSConfiguration = new TalonFXSConfiguration()
            .withSlot0(kHoodSlot0Configs)
            .withCurrentLimits(kHoodCurrentLimitConfig)
            .withMotorOutput(kHoodOutputConfigs)
            .withExternalFeedback(kHoodFeedbackConfigs)
            .withVoltage(kHoodVoltageConfigs)
            .withCommutation(kHoodCommutationConfigs)
            .withSoftwareLimitSwitch(kHoodSoftLimitConfigs);
        public static final TalonFXSConfiguration kHoodTalonFXSConfigurationNoSoftLimit = new TalonFXSConfiguration()
            .withSlot0(kHoodSlot0Configs)
            .withCurrentLimits(kHoodCurrentLimitConfig)
            .withMotorOutput(kHoodOutputConfigs)
            .withExternalFeedback(kHoodFeedbackConfigs)
            .withVoltage(kHoodVoltageConfigs)
            .withCommutation(kHoodCommutationConfigs)
            .withSoftwareLimitSwitch(kHoodSoftLimitConfigsNoEnable);

        //---TURRET
        private static final Slot0Configs kTurretSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(5)
            .withKA(0.02)
            .withKP(300)  //3 - testing values in Pheonix Tuner
            .withKI(0)
            .withKD(5); // OLD: kP was too low making the slope less steep, kS kV and kA were causing rlly weird behavior (jumping up/down way further than targeted position)
        private static final CurrentLimitsConfigs kTurretCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(55)
            .withSupplyCurrentLimit(55)
            .withSupplyCurrentLowerLimit(15)
            .withSupplyCurrentLowerTime(1.0) // drop to 15A after 1 second
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTurretOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        private static final MotionMagicConfigs kTurretMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(110)  //TODO: update MMV Configs
            .withMotionMagicAcceleration(20)
            .withMotionMagicJerk(0);
        private static final SoftwareLimitSwitchConfigs kTurretSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.75)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(-0.75);
        private static final FeedbackConfigs kTurretFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kTurretGearing);
        private static final VoltageConfigs kTurretVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);
        public static final TalonFXConfiguration kTurretTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kTurretSlot0Configs)
            .withCurrentLimits(kTurretCurrentLimitConfigs)
            .withMotorOutput(kTurretOutputConfigs)
            .withMotionMagic(kTurretMotionMagicConfigs)
            .withSoftwareLimitSwitch(kTurretSoftwareLimitSwitchConfigs)
            .withFeedback(kTurretFeedbackConfigs)
            .withVoltage(kTurretVoltageConfigs);

        public static final MagnetSensorConfigs kEncoderAMagnetSensorConfigs = new MagnetSensorConfigs()
            .withMagnetOffset(TurretK.kEncAMagnetOffset);
        public static final CANcoderConfiguration kEncoderAConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(kEncoderAMagnetSensorConfigs);

        //Left, Center (Climb), Center (Hub), Right - Driver POV
        public static final Pose2d kShooterOverridePose[] = {
            AllianceFlipUtil.apply(new Pose2d(FieldK.kFieldLengthMeters / 6, FieldK.kFieldWidthMeters * 2 / 3, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(Units.inchesToMeters(156.61 - 115.05 + 10), FieldK.kFieldWidthMeters / 2, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(Units.inchesToMeters(156.61 - 10), FieldK.kFieldWidthMeters / 2, new Rotation2d(0))),
            AllianceFlipUtil.apply(new Pose2d(FieldK.kFieldLengthMeters / 6, FieldK.kFieldWidthMeters / 3, new Rotation2d(0))),
        };
    }

    public static class VisionK {
        // public static final Camera[] kCameras = new Camera[4];
        // private static final String kSimCameraSimVisualNames = /"VisionEstimation"; //suffixed to each camera name
        // ONSHAPE X IS OUR Y -- ONSHAPE Y IS OUR X !!! NOTE THIS PLEASE DO NOT FORGET
        public static final Transform3d kFrontLeftCTR = VisionUtil.transformToRobo(8.875, 12.18175, 20.45, 180, -20, 45);
        public static final Transform3d kFrontRightCTR = VisionUtil.transformToRobo(8.875, -12.18175, 20.45, 180, -20, -45);
        public static final Transform3d kBackLeftCTR = VisionUtil.transformToRobo(-11.375, 11.875, 20.5625, 0, -20, 135);
        public static final Transform3d kBackRightCTR = VisionUtil.transformToRobo(-12.455, -12.055, 18.25, 180,-20, -135);
        //Initialize cameras
        // static {
        //     kCameras[0] = new Camera(
        //         new SimCameraProperties(), 
        //         "FrontLeft", 
        //         kSimCameraSimVisualNames, 
        //         Camera.transformToRobo(10.413, 12.394, 28.844, 0, -10, 45)
        //     );
        //     kCameras[0].setProps("ThriftyCam", 0, 0, 0, 0);
            
        //     kCameras[1] = new Camera(
        //         new SimCameraProperties(), 
        //         "FrontRight", 
        //         kSimCameraSimVisualNames, 
        //         Camera.transformToRobo(10.413, -12.394, 28.844, 0, -10, 315)
        //     );
        //     kCameras[1].setProps("ThriftyCam", 0, 0, 0, 0);

        //     kCameras[2] = new Camera(
        //         new SimCameraProperties(), 
        //         "BackLeft", 
        //         kSimCameraSimVisualNames, 
        //         Camera.transformToRobo(-11.894, 12.394, 28.844, 0, -10, 135)
        //     );
        //     kCameras[2].setProps("ThriftyCam", 0, 0, 0, 0);

        //     kCameras[3] = new Camera(
        //         new SimCameraProperties(), 
        //         "BackRight", 
        //         kSimCameraSimVisualNames, 
        //         Camera.transformToRobo(-11.894, -12.394, 28.844, 0, -10, 225)
        //     );
        //     kCameras[3].setProps("ThriftyCam", 0, 0, 0, 0);
        // }
    }

    public static class FieldK {
        // take with a grain of salt - pulled from field dimensions (welded)
        public static final double kFieldLengthMeters = Units.inchesToMeters(651.22); 
        public static final double kFieldWidthMeters = Units.inchesToMeters(317.69);

        public static final Pose2d kLeftResetPose = new Pose2d(0.478, 8 - 0.392, Rotation2d.kZero);
        public static final Pose2d kRightResetPose = new Pose2d(0.478, 0.392, Rotation2d.kZero);

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
        public static final String kLogTab = "Robot";

        public static final int kMiniPCChannel = 14;

        // real values
        public static final Distance kRobotFullWidth = Inches.of(33.6875);
        public static final Distance kRobotFullLength = Inches.of(32.6875);
        public static final Distance kBumperHeight = Inches.of(4.5);
        public static final double kRobotSpeedIntakingLimit = 0.31;
        public static final double kRobotEvasionLimit = 1.5;
    }

    public static class SuperstructureK {
        public static final String kLogTab = "Superstructure";
    }

    public static class IntakeK {
        public static final String kLogTab = "Intake";

        /* MOTOR CONSTANTS */
        public static final double kIntakeArmMOI = 0.0209;
        public static final double kIntakeArmGearing = 125/1;

        public static final double kIntakeRollersMOI = 0.0001; // 0.00343880857
        public static final double kIntakeRollersGearing = 12.0/30;

        public static final AngularVelocity kIntakeRollersMaxRPS = MotorK.kX60FOCMaxVelocity.div(kIntakeRollersGearing);
        public static final AngularVelocity kIntakeRollersShootRPS = kIntakeRollersMaxRPS.times(0.2);
        public static final AngularVelocity kIntakeRollersShimmyRPS = kIntakeRollersMaxRPS.times(0.2);
        public static final double kIntakeRollersBarfVolts = -12;
        public static final double kIntakeRollersIntakeVolts = 11;
        public static final double kIntakeRollersShimmyVolts = 5;

        /* IDS */
        public static final int kIntakeArmCANID = 40;
        public static final int kIntakeRollersA_CANID = 41;
        public static final int kIntakeRollersB_CANID = 42;

        /* CONFIGS */
        // IntakeArm Motor
        private static final CurrentLimitsConfigs kIntakeArmCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final Slot0Configs kIntakeArmSlot0Configs = new Slot0Configs()
            .withKS(1.5)
            .withKV(0)
            .withKA(0)
            .withKP(50)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kIntakeArmMotorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        private static final MotionMagicConfigs kIntakeArmMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(64)
            .withMotionMagicJerk(0);
        public static final FeedbackConfigs kIntakeArmFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kIntakeArmGearing);
        private static final VoltageConfigs kIntakeArmVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);
        public static final TalonFXConfiguration kIntakeArmConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeArmCurrentLimitConfigs)
            .withSlot0(kIntakeArmSlot0Configs)
            .withMotorOutput(kIntakeArmMotorOutputConfigs)
            .withMotionMagic(kIntakeArmMotionMagicConfigs)
            .withVoltage(kIntakeArmVoltageConfigs)
            .withFeedback(kIntakeArmFeedbackConfigs);

        // IntakeRollers Motors
        private static final CurrentLimitsConfigs kIntakeRollersACurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(55)
            .withSupplyCurrentLimit(35)
            .withSupplyCurrentLowerTime(0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final Slot0Configs kIntakeRollersASlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(0.048) // 0.488599348534
            .withKA(0)
            .withKP(0.05)
            .withKI(0)
            .withKD(0);
        public static final MotorOutputConfigs kIntakeRollersAMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        public static final FeedbackConfigs kIntakeRollersAFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kIntakeRollersGearing);
        private static final VoltageConfigs kIntakeRollersAVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(12)    //1.2
            .withPeakReverseVoltage(-12);  //-1.2
        public static final TalonFXConfiguration kIntakeRollersAConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(kIntakeRollersACurrentLimitConfigs)
            .withSlot0(kIntakeRollersASlot0Configs)
            .withMotorOutput(kIntakeRollersAMotorOutputConfigs)
            .withFeedback(kIntakeRollersAFeedbackConfigs)
            .withVoltage(kIntakeRollersAVoltageConfigs);

        public static final MotorOutputConfigs kIntakeRollersBMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        public static final TalonFXConfiguration kIntakeRollersBConfiguration = kIntakeRollersAConfiguration.clone()
            .withMotorOutput(kIntakeRollersBMotorOutputConfigs);
    }

    public static class IndexerK {
        public static final String kLogTab = "Indexer";
        
        /* IDS */
        //TODO: Make ids accurate
        public static final int kSpindexerCANID = 10;
        public static final int kTunnelCANID = 11;

        public static final double kSpindexerGearing = 5.0 ; //5
        public static final double kTunnelGearing = 20.0/30.0;

        public static final double kSpindexerMOI = 0.00166190059;
        public static final double kTunnelMOI = 0.000215968064;
      
        public static final AngularVelocity kSpindexerMaxRPS = MotorK.kX60MaxVelocity.div(kSpindexerGearing);
        public static final AngularVelocity kSpindexerIntakeRPS = kSpindexerMaxRPS.times(-0.20);
        public static final AngularVelocity kSpindexerShootRPS = kSpindexerMaxRPS.times(0.85);
        public static final double kSpindexerMaxRPSD    = kSpindexerMaxRPS.in(RotationsPerSecond);
        public static final double kSpindexerShootRPSD = kSpindexerShootRPS.in(RotationsPerSecond);
        public static final double kSpindexerIntakeRPSD = kSpindexerIntakeRPS.in(RotationsPerSecond);

        public static final AngularVelocity kTunnelMaxRPS = MotorK.kX60FOCMaxVelocity.div(kTunnelGearing);
        public static final AngularVelocity kTunnelShootRPS = kTunnelMaxRPS.times(0.77);    //9V
        public static final double kTunnelMaxRPSD    = kTunnelMaxRPS.in(RotationsPerSecond);
        public static final double kTunnelShootRPSD = kTunnelShootRPS.in(RotationsPerSecond);

        public static final AngularVelocity kTunnelSpunUpMinimum = RotationsPerSecond.of(10);
        public static final double kTunnelSpunUpMinimumD = 10.0;
        public static final Time kTunnelSpunUpTimeout = Seconds.of(1);  //double expected spinup time

        /* VELOCITY RATIO (shooter RPS → indexer RPS) */
        public static final double kR_bigFlywheel    = ShooterK.kFlywheelRadiusM; // 0.0381 m
        public static final double kR_smallFlywheel  = 0.0215;  // m
        public static final double kR_tunnelPulley   = 0.018;   // m
        public static final double kR_spindexerFloor = 6.5 * 0.0254; // 0.1651 m

        public static final double kTunnelFromShooterRatio    = (kR_bigFlywheel + kR_smallFlywheel) / (2.0 * kR_tunnelPulley);
        public static final double kSpindexerFromShooterRatio = (kR_bigFlywheel + kR_smallFlywheel) / (2.0 * kR_spindexerFloor);

        /* CONFIGS */
        //TODO: Make transfer configs accurate
        private static final Slot0Configs kSpindexerSlot0Configs = new Slot0Configs()
            .withKS(0.420)
            .withKV(0.560)
            .withKA(0)
            .withKP(1.5)
            .withKI(0)
            .withKD(0);
        private static final CurrentLimitsConfigs kSpindexerCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(65)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLowerTime(0)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kSpindexerMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kSpindexerFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kSpindexerGearing);
        private static final VoltageConfigs kSpindexerVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(16)    //1.2
            .withPeakReverseVoltage(-16);  //-1.2
        public static final TalonFXConfiguration kSpindexerTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kSpindexerSlot0Configs)
            .withCurrentLimits(kSpindexerCurrentLimitConfigs)
            .withMotorOutput(kSpindexerMotorOutputConfigs)
            .withFeedback(kSpindexerFeedbackConfigs)
            .withVoltage(kSpindexerVoltageConfigs);

        private static final Slot0Configs kTunnelSlot0Configs = new Slot0Configs()
            .withKS(0.2)
            .withKV(0.086)
            .withKA(0)
            .withKP(0.37)
            .withKI(0)
            .withKD(0);
        private static final CurrentLimitsConfigs kTunnelCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLowerTime(0)
            .withSupplyCurrentLowerLimit(20)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTunnelMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        private static final FeedbackConfigs kTunnelFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(kTunnelGearing);
        private static final VoltageConfigs kTunnelVoltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(16)  //1.2
            .withPeakReverseVoltage(-16);    //-1.2
        public static final TalonFXConfiguration kTunnelTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kTunnelSlot0Configs)
            .withCurrentLimits(kTunnelCurrentLimitConfigs)
            .withMotorOutput(kTunnelMotorOutputConfigs)
            .withFeedback(kTunnelFeedbackConfigs)
            .withVoltage(kTunnelVoltageConfigs);
    }

    public static class TurretK {
        public static final String kLogTab = "Turret";
        
        public static final double kGearZeroToothCount = 100;
        public static final double kGearOneToothCount = 10;
        public static final double kGearTwoToothCount = 19;

        public static final double kLCMAtHomeRots = 0.251; // measure: turretLCMPos log value when turret is at home
        public static final double kEncAMagnetOffset = 0.320556640625;
        public static final double kEncBOffset = 0.529614; // measure: encB reading when turret is at encA=0 //NEW ONE
    }
    public static class AutonK {
        public static final String kLogTab = "Auton";

        public static final Pose2d kRightNeutralPose = new Pose2d(Meters.of(6.924767017364502), 
            Meters.of(2.251265048980713), new Rotation2d(0));
        public static final Pose2d kRightDepotPose = new Pose2d(Meters.of(1.1576627492904663), 
            Meters.of(5.958622932434082), new Rotation2d(Math.PI));

        public static final Pose2d kLeftNeutralPose = new Pose2d(Meters.of(6.924767017364502), 
            Meters.of(5.437880039215088), new Rotation2d(0));

        public static final double kIntakeTimeout = 7.5;
        public static final double kShootingTimeout = 4; //12
        public static final double kSOTMTimeout = 100; //12
        public static final double kSweepShootingTimeout = 20;

        public static final double kFollowDelay = 2;

        /* OLD PATHS */
        //---RIGHT FIRST CYCLES
        public static final String kRightOneJab = "RIGHT_one_jab";
        public static final String kRightOneTrench = "RIGHT_one_trench";
        public static final String kRightOneDefense = "RIGHT_one_defense";
        public static final String kRightOneReverse = "RIGHT_one_reverse";

        //---RIGHT SECOND CYCLES
        public static final String kRightTwoSotmDepot = "RIGHT_two_sotmDepot";
        public static final String kRightTwoDepot = "RIGHT_two_depot";
        public static final String kRightTwoSweep = "RIGHT_two_sweep";
        public static final String kRightTwoPassing = "RIGHT_two_passing";
        public static final String kRightTwoJab = "RIGHT_two_jab";
        public static final String kRightTwoReverse = "RIGHT_two_reverse";

        //---LEFT FIRST CYCLES
        public static final String kLeftOneJab = "LEFT_one_jab";
        public static final String kLeftOneTrench = "LEFT_one_trench";
        public static final String kLeftOneDefense = "LEFT_one_defense";
        public static final String kLeftOneReverse = "LEFT_one_reverse";

        //---LEFT SECOND CYCLES
        public static final String kLeftTwoSotmDepot = "LEFT_two_sotmDepot";
        public static final String kLeftTwoDepot = "LEFT_two_depot";
        public static final String kLeftTwoSweep = "LEFT_two_sweep";
        public static final String kLeftTwoPassing = "LEFT_two_passing";
        public static final String kLeftTwoJab = "LEFT_two_jab";
        public static final String kLeftTwoReverse = "LEFT_two_reverse";

        //---MISC
        public static final String kRightOneCircle = "RIGHT_one_circle";
        public static final String kLeftOneSweepAndDepot = "LEFT_one_sweepAndDepot";
        public static final String kLeftThreeDepotToBump = "LEFT_three_depotToBump";
        public static final String kRightThreeDepotToBump = "RIGHT_three_depotToBump";

        //---STRESS TEST
        public static final String kRightStressTestLong = "RIGHT_stress_test_long";
        public static final String kRightStressTestOverlap = "RIGHT_stress_test_overlap";
      
        /* NEW PATHS */
        //---BUMP RETURN PATHS
        public static final String kRightOneBumpReturn = "RIGHT_one_bumpReturn";
        public static final String kRightOneBumpReturnFollow = "RIGHT_one_bumpReturnFollow";
        public static final String kLeftOneBumpReturn = "LEFT_one_bumpReturn";
        public static final String kLeftOneBumpReturnFollow = "LEFT_one_bumpReturnFollow";
        public static final String kRightTwoBumpReturn = "RIGHT_two_bumpReturn";
        public static final String kLeftTwoBumpReturn = "LEFT_two_bumpReturn";
        public static final String kRightTwoBumpToTrench = "RIGHT_two_bumpToTrench";
        public static final String kLeftTwoBumpToTrench = "LEFT_two_bumpToTrench";

        //---TRENCH RETURN PATHS
        public static final String kRightOneTrenchReturn = "RIGHT_one_trenchReturn";
        public static final String kLeftOneTrenchReturn = "LEFT_one_trenchReturn";
        public static final String kRightTwoTrenchReturn = "RIGHT_two_trenchReturn";
        public static final String kLeftTwoTrenchReturn = "LEFT_two_trenchReturn";
        
        //---OUTPOST PATHS
        public static final String kRightOneTrenchToOutpost = "RIGHT_one_trenchToOutpost";
        public static final String kRightTwoTrenchToOutpost = "RIGHT_two_trenchToOutpost";
        public static final String kRightTwoOutpostToTrench = "RIGHT_two_outpostToTrench";
        public static final String kRightOneBumpToOutpost = "RIGHT_one_bumpToOutpost";
        public static final String kRightTwoBumpToOutpost = "RIGHT_two_bumpToOutpost";
        public static final String kRightTwoOutpostToBump = "RIGHT_two_outpostToBump";

        //---DEPOT PATHS
        public static final String kLeftOneTrenchToDepot = "LEFT_one_trenchToDepot";
        public static final String kLeftTwoTrenchToDepot = "LEFT_two_trenchToDepot";
        public static final String kLeftTwoDepotToTrench = "LEFT_two_depotToTrench";
        public static final String kLeftOneBumpToDepot = "LEFT_one_bumpToDepot";
        public static final String kLeftTwoBumpToDepot = "LEFT_two_bumpToDepot";
        public static final String kLeftTwoDepotToBump = "LEFT_two_depotToBump";

        //---MISC
        public static final String kRightOneSelfPass = "RIGHT_one_selfPass";
        public static final String kLeftOneSelfPass = "LEFT_one_selfPass";
        public static final String kRightTwoGoOut = "RIGHT_two_goOut";
        public static final String kLeftTwoGoOut = "LEFT_two_goOut";
        public static final String kRightBumpPreload = "RIGHT_one_bumpPreload";
        public static final String kLeftBumpPreload = "LEFT_one_bumpPreload";
        public static final String kRightTrenchPreload = "RIGHT_one_trenchPreload";
        public static final String kLeftTrenchPreload = "LEFT_one_trenchPreload";
    }
}
