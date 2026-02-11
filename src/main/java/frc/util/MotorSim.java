package frc.util;

import static frc.robot.Constants.ShooterK.kTurretGearing;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class MotorSim {
    public static void initSimFX(TalonFX motor, ChassisReference chassisReference, TalonFXSimState.MotorType motorType) {
        var motorFXSim = motor.getSimState();
        motorFXSim.Orientation = chassisReference;
        motorFXSim.setMotorType(motorType);
    }

    public static void updateSimFX(TalonFX motor, DCMotorSim motorSim) {
        var motorFXSimState = motor.getSimState();

        motorSim.setInputVoltage(motorFXSimState.getMotorVoltage());
        motorSim.update(Constants.kSimPeriodicUpdateInterval);

        motorFXSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * motorSim.getGearing());
        motorFXSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorSim.getGearing()));
        motorFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    public static void updateSimFX(TalonFX motor, FlywheelSim motorSim) {
        var motorFXSimState = motor.getSimState();

        motorSim.setInputVoltage(motorFXSimState.getMotorVoltage());
        motorSim.update(Constants.kSimPeriodicUpdateInterval);

        motorFXSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorSim.getGearing()));
        motorFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
