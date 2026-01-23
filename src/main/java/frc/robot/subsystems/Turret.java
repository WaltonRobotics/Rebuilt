package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.units.*;

public class Turret {

    private final TalonFX m_motor = new TalonFX(11);
    
    // Motor Positions (for max/min assume exclusive)
    private final double kMaxPos = 135;
    private final double kMinPos = -821;
    private final double kHomingPos = 0;
    private final double kFrontFacingPos = -330;
    private final double kUnitsPerRotation = 1360; // 1360 units per rotation

    private double currentPos;

    public Turret() {
        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        moveToPosRotations(kHomingPos);
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
    public boolean moveToPosMysteryUnits(double pos) {
        if (kMinPos < pos && kMaxPos > pos) {
            currentPos = pos;
            StatusCode code = m_motor.setControl(new MotionMagicVoltage(pos / kUnitsPerRotation));
            return code == StatusCode.OK;
        } 
        else {
            return false;
        }
    }

    /**
     * Rotates turret to specified position.
     * @param pos Position in rotations
     * @return If rotation was successful
     */
    public boolean moveToPosRotations(double rots) {
        if (kMinPos / kUnitsPerRotation < rots && kMaxPos / kUnitsPerRotation > rots) {
            currentPos = rots * kUnitsPerRotation;
            StatusCode code = m_motor.setControl(new MotionMagicVoltage(rots));
            return code == StatusCode.OK;
        } 
        else {
            return false;
        }
    }

    public boolean moveToMaxMysteryUnits() {
        return moveToPosMysteryUnits(kMaxPos - 0.1);
    }

    public boolean moveToMinMysteryUnits() {
        return moveToPosMysteryUnits(kMinPos + 0.1);
    }

    public boolean moveToHomeMysteryUnits() {
        return moveToPosMysteryUnits(kHomingPos);
    }

    public boolean moveToFrontFacingMysteryUnits() {
        return moveToPosMysteryUnits(0);
    }

    public boolean moveToMaxRotations() {
        return moveToPosRotations(kMaxPos - 0.1);
    }

    public boolean moveToMinRotations() {
        return moveToPosRotations(kMinPos + 0.1);
    }

    public boolean moveToHomeRotations() {
        return moveToPosRotations(kHomingPos);
    }

    public boolean moveToFrontFacingRotations() {
        return moveToPosRotations(kFrontFacingPos);
    }
}