package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public class ShooterK {
        private static final Slot0Configs kTurretSlot0Configs = new Slot0Configs()
            .withKS(0)
            .withKV(4.07)
            .withKA(0.02)
            .withKP(120)  //3 - testing values in Pheonix Tuner
            .withKI(0)
            .withKD(0); // OLD: kP was too low making the slope less steep, kS kV and kA were causing rlly weird behavior (jumping up/down way further than targeted position)
        private static final CurrentLimitsConfigs kTurretCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimitEnable(true);
        private static final MotorOutputConfigs kTurretOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //TODO: check whether this should be CW or CCW
            .withNeutralMode(NeutralModeValue.Brake)
            .withPeakForwardDutyCycle(0.1)
            .withPeakReverseDutyCycle(0.1);
        private static final MotionMagicConfigs kTurretMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(110)  //TODO: update MMV Configs
            .withMotionMagicAcceleration(20)
            .withMotionMagicJerk(0);
        private static final SoftwareLimitSwitchConfigs kTurretSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(0.78)    //TODO: update threshold numbers
            .withReverseSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(0.02);
        private static final FeedbackConfigs kTurretFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(41.666666666);
        public static final TalonFXConfiguration kTurretTalonFXConfiguration = new TalonFXConfiguration()
            .withSlot0(kTurretSlot0Configs)
            .withCurrentLimits(kTurretCurrentLimitConfigs)
            .withMotorOutput(kTurretOutputConfigs)
            .withMotionMagic(kTurretMotionMagicConfigs)
            .withSoftwareLimitSwitch(kTurretSoftwareLimitSwitchConfigs)
            .withFeedback(kTurretFeedbackConfigs);
    }
}
