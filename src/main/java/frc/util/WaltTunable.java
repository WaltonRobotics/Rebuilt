package frc.util;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.DoubleConsumer;

/**
 * A NetworkTables-tunable double with an enable/disable toggle.
 * Uses NT listeners (no polling) and caches values in volatile fields for zero-overhead reads.
 *
 * <p>Usage:
 * <pre>
 * private static final WaltTunable kMyGain = new WaltTunable("/ShotCalc/myGain", 0.005);
 *
 * // In hot path:
 * if (kMyGain.enabled()) {
 *     double val = kMyGain.get();
 * }
 * </pre>
 *
 * <p>Two NT entries are created:
 * <ul>
 *   <li>{@code <key>/value} — the tunable double value</li>
 *   <li>{@code <key>/enabled} — boolean toggle (default false)</li>
 * </ul>
 */
public class WaltTunable {
    private volatile double m_value;
    private volatile boolean m_enabled;
    private final List<DoubleConsumer> m_onChange = new CopyOnWriteArrayList<>();

    // Must hold references to prevent GC from killing the listeners
    private final DoubleEntry m_valueEntry;
    private final BooleanEntry m_enabledEntry;

    public WaltTunable(String key, double defaultValue) {
        this(key, defaultValue, false);
    }

    public WaltTunable(String key, double defaultValue, boolean defaultEnabled) {
        m_value = defaultValue;
        m_enabled = defaultEnabled;

        var inst = NetworkTableInstance.getDefault();
        var kinds = EnumSet.of(NetworkTableEvent.Kind.kValueRemote);

        m_valueEntry = inst.getDoubleTopic(key + "/value").getEntry(defaultValue);
        m_valueEntry.setDefault(defaultValue);
        inst.addListener(m_valueEntry, kinds, e -> {
            m_value = e.valueData.value.getDouble();
            fireOnChange();
        });

        m_enabledEntry = inst.getBooleanTopic(key + "/enabled").getEntry(defaultEnabled);
        m_enabledEntry.setDefault(defaultEnabled);
        inst.addListener(m_enabledEntry, kinds, e -> {
            m_enabled = e.valueData.value.getBoolean();
            fireOnChange();
        });
    }

    private void fireOnChange() {
        double val = m_value;
        for (var cb : m_onChange) {
            cb.accept(val);
        }
    }

    /** Current value (volatile read, no NT overhead). */
    public double get() {
        return m_value;
    }

    /** Whether tuning is enabled (volatile read, no NT overhead). */
    public boolean enabled() {
        return m_enabled;
    }

    /** Convenience: returns the value if enabled, otherwise the provided fallback. */
    public double getOr(double fallback) {
        return m_enabled ? m_value : fallback;
    }

    /**
     * Register a callback that fires only when the value or enabled state changes remotely.
     * The callback receives the current value. Useful for expensive operations that
     * should not be polled.
     */
    public WaltTunable onChange(DoubleConsumer callback) {
        m_onChange.add(callback);
        return this;
    }
}
