// package frc.robot.subsystems;

// import javax.sound.sampled.Control;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.units.AngleUnit;

// import com.ctre.phoenix6.configs.*;
// import com.ctre.phoenix6.controls.*;

// import edu.wpi.first.units.*;


// public class Turret {

//     //  ---------------- Constants --------------------
    
//     // Motor stuff
//     private final TalonFX m_motor = new TalonFX(11);
//     // private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
//     // private final TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
    
//     // Motor Positions (for max/min assume exclusive)
//     private final double kMaxPos = 428;
//     private final double kMinPos = -452;
//     private final double kHomingPos = 342;
//     private double currentPos;

//     public Turret() {
//         // TODO apply config

//         m_motor.getConfigurator().apply(new TalonFXConfiguration());

//         currentPos = kHomingPos;
//         m_motor.setPosition(0);
//         moveToPos(currentPos);
        
//     }

//     private double getPos() {
//         return m_motor.getPosition().getValue().abs(Units.Rotations);
//     }
    
//     public boolean moveToPos(double pos) {
//         // TODO get current pos for reasons unknown
//         if (kMinPos < pos && kMaxPos > pos) {
//             currentPos = pos;
//             m_motor.setPosition(currentPos);
//             return true;
//         } 
//         else {
//             return false;
//         }
//     }

//     public boolean moveToMax() {
//         return moveToPos(kMaxPos - 0.1);
//     }

//     public boolean moveToMin() {
//         return moveToPos(kMinPos + 0.1);
//     }

//     public boolean moveToHome() {
//         return moveToPos(kHomingPos);
//     }
// }