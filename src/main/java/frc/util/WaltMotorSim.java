package frc.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Motor simulation methods */
public class WaltMotorSim {
    /**
     * Initialize the sim for a CTRE TalonFX motor
     * @param motor is the CTRE motor
     * @param chassisReference is the orientation of the device relative to the robot chassis
     * @param motorType is the type of motor (ex. Kraken X60)
     */
    public static void initSimFX(TalonFX motor, ChassisReference chassisReference, TalonFXSimState.MotorType motorType) {
        var motorFXSim = motor.getSimState();
        motorFXSim.Orientation = chassisReference;
        motorFXSim.setMotorType(motorType);
    }

    /**
     * Update the DCMotorSim for a CTRE TalonFX motor
     * @param motor is the CTRE motor
     * @param motorSim is the corresponding DCMotorSim object for the CTRE motor
     */
    public static void updateSimFX(TalonFX motor, DCMotorSim motorSim) {
        var motorFXSimState = motor.getSimState();

        motorSim.setInputVoltage(motorFXSimState.getMotorVoltage());
        motorSim.update(Constants.kSimPeriodicUpdateInterval);

        motorFXSimState.setRawRotorPosition(motorSim.getAngularPositionRotations() * motorSim.getGearing());
        motorFXSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorSim.getGearing()));
        motorFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    /**
     * Update the FlywheelSim for a CTRE TalonFX motor
     * @param motor is the CTRE motor
     * @param motorSim is the corresponding FlywheelSim object for the CTRE motor
     */
    public static void updateSimFX(TalonFX motor, FlywheelSim motorSim) {
        var motorFXSimState = motor.getSimState();

        motorSim.setInputVoltage(motorFXSimState.getMotorVoltage());
        motorSim.update(Constants.kSimPeriodicUpdateInterval);

        motorFXSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorSim.getGearing()));
        motorFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    /**
     * Update the DCMotorSim for a servo
     * 
     * Designed for the hood servo, but can be used by other servos
     * @param servo must act like a DCMotor for this to work
     * @param motorSim is the corresponding DCMotorSim object for the DCMotor-like servo
     */
    public static void updateSimServo(GobildaServoContinuous servo, DCMotorSim motorSim) {
        double volts = servo.get();
        volts -= 0.5; // sets the range from [0, 1] to be [-0.5, 0.5]
        volts *= 2; // sets the range from [-0.5, 0.5] to be [-1, 1]
        volts *= motorSim.getGearbox().nominalVoltageVolts; // applies the volts sent to the servo on a [-6, 6] range

        motorSim.setInputVoltage(volts);
        motorSim.update(Constants.kSimPeriodicUpdateInterval);
    }
}
