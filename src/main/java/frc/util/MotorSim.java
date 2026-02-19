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
public class MotorSim {
    /**
     * Initializes a motor simulation. 
     * @param motor The motor to simulate.
     * @param chassisReference The orientation of the motor.
     * @param motorType The simulated motor type (X44 or X60)
     */
    public static void initSimFX(TalonFX motor, ChassisReference chassisReference, TalonFXSimState.MotorType motorType) {
        var motorFXSim = motor.getSimState();
        motorFXSim.Orientation = chassisReference;
        motorFXSim.setMotorType(motorType);
    }
    /**
     * Updates a DC motor simulation
     * @param motor The motor to simulate.
     * @param motorSim A DC motor simulation.
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
     * Updates a Flywheel motor simulation.
     * @param motor The motor to simulate.
     * @param motorSim A simulated flywheel.
     */
    public static void updateSimFX(TalonFX motor, FlywheelSim motorSim) {
        var motorFXSimState = motor.getSimState();

        motorSim.setInputVoltage(motorFXSimState.getMotorVoltage());
        motorSim.update(Constants.kSimPeriodicUpdateInterval);

        motorFXSimState.setRotorVelocity(motorSim.getAngularVelocity().times(motorSim.getGearing()));
        motorFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

   /**
     * Updates a servo motor simulation.
     * This is designed for the hood servo, but it can be used by other servos.
     * @param servo The servo to simulate.
     * @param motorSim A DC motor simulation.
     */
    public static void updateSimServo(Servo servo, DCMotorSim motorSim) {
        double volts = servo.get();
        volts -= 0.5; // sets the range from [0, 1] to be [-0.5, 0.5]
        volts *= 2; // sets the range from [-0.5, 0.5] to be [-1, 1]
        volts *= motorSim.getGearbox().nominalVoltageVolts; // applies the volts sent to the servo on a [-6, 6] range

        motorSim.setInputVoltage(volts);
        motorSim.update(Constants.kSimPeriodicUpdateInterval);
    }
}
