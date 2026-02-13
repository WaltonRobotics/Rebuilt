package frc.util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableNumber implements DoubleSupplier{
    private static final String kTableKey = "/Tuning";

    private String key;
    private boolean hasDefault = false;
    private double defaultVal;
    private LoggedNetworkNumber dashboardNumber;
    private Map<Integer, Double> lastHasChangedVals = new HashMap<>();

    public LoggedTunableNumber(String dashboardKey) {
        this.key = kTableKey + "/" + dashboardKey;
    }

    public LoggedTunableNumber(String dashboardKey, double defaultVal) {
        this(dashboardKey);
        initDefault(defaultVal);
    }

    private void initDefault(double defaultVal) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultVal = defaultVal;
            if (Constants.tuningMode && !Constants.disableHAL) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultVal);
            }
        }
    }

    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return Constants.tuningMode && !Constants.disableHAL ? dashboardNumber.get() : defaultVal;
        }
    }

    public boolean hasChanged(int id) {
        double currentVal = get();
        Double lastVal = lastHasChangedVals.get(id);
        if (lastVal == null || currentVal != lastVal) {
            lastHasChangedVals.put(id, currentVal);
            return true;
        }
        return false;
    }

    public static void ifChanged(int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
        }
    }

    public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
        ifChanged(id, values -> action.run(), tunableNumbers);
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
