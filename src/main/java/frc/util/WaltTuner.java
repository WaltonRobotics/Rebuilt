package frc.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public final class WaltTuner {
    /**
     * creates a boolean toggle switch
     * @param tab for the widget
     * @param name of the widget
     * @param defaultVal of the widget
     * @return the widget
     */
    public static GenericEntry createBoolToggleSwitch(String tab, String name, boolean defaultVal) {
        return Shuffleboard.getTab(tab)
        .add(name, defaultVal)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();
    }

    /**
     * creates a boolean toggle button
     * @param tab for the widget
     * @param name of the widget
     * @param defaultVal of the widget
     * @return the widget
     */
    public static GenericEntry createBoolToggleButton(String tab, String name, boolean defaultVal) {
        return Shuffleboard.getTab(tab)
        .add(name, defaultVal)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
    }

    //---UTIL METHODS
    /**
     * Toggles the motor in coast or brake
     * @param coastStateHolder is the motor in coast
     * @param coastRequest desired NeutralModeValue from dashboard widget
     * @param motor the motor you want to toggle coast
     */
    public static boolean toggleMotorCoast(boolean coastStateHolder, boolean coastRequest, TalonFX motor) {
        if (coastRequest != coastStateHolder) {
            motor.setNeutralMode(coastRequest ? NeutralModeValue.Coast : NeutralModeValue.Brake);
            return coastRequest;
        }
        return coastStateHolder;
    }
}
