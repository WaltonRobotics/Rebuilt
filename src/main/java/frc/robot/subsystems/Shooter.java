package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterPrimary = new TalonFX(20);
    private final TalonFX shooterSecondary = new TalonFX(21);

    // private double shooterSpeed = 0.0;

    public Shooter() {
        shooterSecondary.setControl(new Follower(20, MotorAlignmentValue.Opposed));
    }

    public Command shoot(DoubleSubscriber sub_Shooter) {
        return runEnd(() -> {
            shooterPrimary.set(sub_Shooter.get(0));
        }, () -> {
            shooterPrimary.set(0);
        });
    }

    // public Command shoot(DoubleSupplier dutycycle) {
    //     return runEnd(() -> {
    //         shooterPrimary.set(shooterSpeed);
    //     }, () -> {
    //         shooterPrimary.set(0);
    //     });
    // }
    // 
    // public void upShooterSpeed() {
    //     shooterSpeed += 0.05;
    // }

    // public void lowerShooterSpeed() {
    //     shooterSpeed -= 0.05;
    // }
}
