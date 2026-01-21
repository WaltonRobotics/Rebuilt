// package frc.robot.subsystems;
// import java.util.function.DoubleConsumer;
// import java.util.function.DoubleSupplier;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Drive extends SubsystemBase {
//     private static final SparkMax frontLeft = new SparkMax(2, MotorType.kBrushless);
//     private static final SparkMax frontRight = new SparkMax(4, MotorType.kBrushless);
//     private static final SparkMax backLeft = new SparkMax(1, MotorType.kBrushless);
//     private static final SparkMax backRight = new SparkMax(3, MotorType.kBrushless);

//     // private static final DifferentialDrive diffDrive = new DifferentialDrive(
//         // frontLeft::set, frontRight::set
//     // );

//     public Drive() {
//         // diffDrive.setDeadband(0.2);

//         frontLeft.configure(
//             new SparkMaxConfig()
//                 .idleMode(IdleMode.kBrake)
//                 .inverted(false),
//             ResetMode.kResetSafeParameters,
//             PersistMode.kPersistParameters
//         );

//         frontRight.configure(
//             new SparkMaxConfig()
//                 .idleMode(IdleMode.kBrake)
//                 .inverted(true), 
//             ResetMode.kResetSafeParameters,
//             PersistMode.kPersistParameters
//         );

//         backLeft.configure(
//             new SparkMaxConfig()
//                 .follow(frontLeft),
//             ResetMode.kResetSafeParameters,
//             PersistMode.kPersistParameters
//         );

//         backRight.configure(
//             new SparkMaxConfig()
//                 .follow(frontRight),
//             ResetMode.kResetSafeParameters,
//             PersistMode.kPersistParameters
//         );
//     }

//     public Command driveFrontLeft(double dir) {
//         return runEnd(() -> {
//             frontLeft.set(dir);
//         },
//         () -> {
//             frontLeft.set(0);
//         });
//     }

//     public Command driveFrontRight(double dir) {
//         return runEnd(() -> {
//             frontRight.set(dir);
//         },
//         () -> {
//             frontRight.set(0);
//         });
//     }

//     public Command drive(DoubleSupplier x, DoubleSupplier rot) {
//         return run(() -> {
//             // diffDrive.arcadeDrive(x.getAsDouble(), rot.getAsDouble());
//         });
//     }

// }
