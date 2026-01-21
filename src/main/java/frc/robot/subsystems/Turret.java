// package frc.robot.subsystems;

// import javax.sound.sampled.Control;

// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;


// public class Turret {
//     private final TalonSRX motor = new TalonSRX(11);

//     // Constants
//     private final double maxPos = 428;
//     private final double minPos = -452;
//     private final double homingPos = 342;

//     private double currentPos;

//     public Turret() {
//         currentPos = homingPos;

//         motor.configAllSettings(new TalonSRXConfiguration());

//         motor.moveToPos(homingPos);
        
//     }

//     private void moveToPos(double pos) {
//         currentPos = motor.getSelectedSensorPosition();

//         motor.set(TalonSRXControlMode.Position, pos);
//     }
// }
