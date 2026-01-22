package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.units.*;

public class Turret {

    private final TalonFX m_motor = new TalonFX(11);
    
    // Motor Positions (for max/min assume exclusive)
    private final double kMaxPos = 428;
    private final double kMinPos = -452;
    private final double kHomingPos = 342;
    private double currentPos;

    public Turret() {
        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        moveToPos(kHomingPos);
    }

    public double getPosRotations() {
        return m_motor.getPosition().getValue().in(Units.Rotations);
    }

    public double getPosMysteryUnits() { // placeholder name bc we dont know the units
        return currentPos; 
    } 
    
    public boolean moveToPos(double pos) {
        if (kMinPos < pos && kMaxPos > pos) {
            currentPos = pos;
            StatusCode code = m_motor.setControl(new MotionMagicVoltage(currentPos / 360)); // 360 is a placeholder, i dont know what unit the constants are in
            return code == StatusCode.OK;
        } 
        else {
            return false;
        }
    }

    public boolean moveToMax() {
        return moveToPos(kMaxPos - 0.1);
    }

    public boolean moveToMin() {
        return moveToPos(kMinPos + 0.1);
    }

    public boolean moveToHome() {
        return moveToPos(kHomingPos);
    }

    public boolean moveToZero() {
        return moveToPos(0);
    }
}