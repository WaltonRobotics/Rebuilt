package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final TalonFX m_motor = new TalonFX(11);

    private final MotionMagicVoltage m_MMVReq = new MotionMagicVoltage(0);
    
    // Motor Positions (for max/min assume exclusive) NOT rotations (mystery units)
    private final double kMaxPos = 135;
    private final double kMinPos = -821;
    public final double kHomingPos = 0;
    private final double kFrontFacingPos = -330;
    private final double kUnitsPerRotation = 1360; // 1360 units per rotation

    private double currentPos;

    public Turret() {
        m_motor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withPeakForwardDutyCycle(0.1)
                .withPeakReverseDutyCycle(0.1)
            )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withForwardSoftLimitThreshold(Rotations.of(0.78))
                .withReverseSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(Rotations.of(0.02))
            )
        );
    }

    public double getPosRotations() {
        return m_motor.getPosition().getValue().in(Units.Rotations);
    }

    public double getPosMysteryUnits() { // placeholder name bc we dont know the units
        return currentPos; 
    } 
    
    /**
     * Rotates turret to specified position.
     * @param pos Position in [mystery unit]
     * @return If rotation was successful
     */
    public void moveToPosMysteryUnits(double pos) {
        if (kMinPos < pos && kMaxPos > pos) {
            currentPos = pos;
            m_motor.setControl(m_MMVReq.withPosition(pos / kUnitsPerRotation));
        }
    }

    /**
     * Rotates turret to specified position.
     * @param pos Position in rotations
     * @return If rotation was successful
     */
    public void moveToPosRotations(double rots) {
        if (kMinPos / kUnitsPerRotation < rots && kMaxPos / kUnitsPerRotation > rots) {
            currentPos = rots * kUnitsPerRotation;
            m_motor.setControl(m_MMVReq.withPosition(rots));
        }
    }

    public Command spin(DoubleSubscriber sub_Turret) {
        return run(() -> {
            moveToPosMysteryUnits(sub_Turret.getAsDouble());
        });
    }

    public void moveToMaxMysteryUnits() {
        moveToPosMysteryUnits(kMaxPos - 0.1);
    }

    public void moveToMinMysteryUnits() {
        moveToPosMysteryUnits(kMinPos + 0.1);
    }

    public void moveToHomeMysteryUnits() {
        moveToPosMysteryUnits(kHomingPos);
    }

    public void moveToFrontFacingMysteryUnits() {
        moveToPosMysteryUnits(0);
    }

    public void moveToMaxRotations() {
        moveToPosRotations(kMaxPos - 0.1);
    }

    public void moveToMinRotations() {
        moveToPosRotations(kMinPos + 0.1);
    }

    public void moveToHomeRotations() {
        moveToPosRotations(kHomingPos);
    }

    public void moveToFrontFacingRotations() {
        moveToPosRotations(kFrontFacingPos);
    }
}