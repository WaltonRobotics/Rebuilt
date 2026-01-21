// package frc.robot.subsystems;

// import javax.sound.sampled.Control;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.configs.*;
// import com.ctre.phoenix6.controls.*;


// public class Turret {

//     //  ---------------- Constants --------------------

    
//     // Motor stuff
//     private final TalonFX m_motor = new TalonFX(11);
//     private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
//     private final TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
    
//     // Motor Positions (for max/min assume exclusive)
//     private final double kMaxPos = 428;
//     private final double kMinPos = -452;
//     private final double kHomingPos = 342;
//     private double currentPos;

//     public Turret() {
//         currentPos = kHomingPos;
//         // TODO apply config
//         m_motor.setPosition(currentPos);
        
//     }

//     private boolean moveToPos(double pos) {
//         // TODO get current pos for reasons unknown
//         if (kMinPos < pos && kMaxPos > pos){
//             m_motor.setPosition(pos);
//             return true;
//         } else {
//             return false;
//         }


//     }
// }