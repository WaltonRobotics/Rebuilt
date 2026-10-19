package frc.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.system.RobotController;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.FlywheelSim;
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
    // 2027-TODO: fix this!!!
    public static void updateSimFX(TalonFX motor, DCMotorSim motorSim) {
    //     var motorFXSimState = motor.getSimState();

    //     motorSim.setInputVoltage(motorFXSimState.getMotorVoltage());
    //     motorSim.update(Constants.kSimPeriodicUpdateInterval);

    //     motorFXSimState.setRawRotorPosition(motorSim.getAngularPosition() * motorSim.getGearing());
    //     motorFXSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorSim.getGearing()));
    //     motorFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    /**
     * Update the FlywheelSim for a CTRE TalonFX motor
     * @param motor is the CTRE motor
     * @param motorSim is the corresponding FlywheelSim object for the CTRE motor
     */
    // 2027-TODO: fix!!!
    //// error: double cannot be dereferenced
    //// motorFXSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorSim.getGearing()));
    public static void updateSimFX(TalonFX motor, FlywheelSim motorSim) {
        var motorFXSimState = motor.getSimState();

        motorSim.setInputVoltage(motorFXSimState.getMotorVoltage());
        motorSim.update(Constants.kSimPeriodicUpdateInterval);

        // motorFXSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorSim.getGearing()));
        motorFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}
