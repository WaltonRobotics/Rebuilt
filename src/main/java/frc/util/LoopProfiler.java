package frc.util;

import java.util.ArrayList;
import java.util.List;

import frc.util.WaltLogger.DoubleLogger;

/**
 * Tracer-style per-section timer. Sections are pre-registered at construction so the
 * hot path is a flat indexed array access (no HashMap lookup, no allocation per tick).
 *
 * Usage:
 *   m_profiler = new LoopProfiler("Perf/Loop", "signals", "scheduler", "vision", "logging");
 *   ...
 *   m_profiler.start();
 *   SignalManager.refreshAll();          m_profiler.mark(0);
 *   CommandScheduler.getInstance().run(); m_profiler.mark(1);
 *   doVision();                           m_profiler.mark(2);
 *   doLogging();                          m_profiler.mark(3);
 */
public final class LoopProfiler {
    private final DoubleLogger[] m_loggers;
    private long m_lastNanos;

    public LoopProfiler(String tab, String... sections) {
        m_loggers = new DoubleLogger[sections.length];
        for (int i = 0; i < sections.length; i++) {
            m_loggers[i] = WaltLogger.logDouble(tab, sections[i] + "Ms");
        }
    }

    /** Reset the section timer. Call once at the top of the periodic. */
    public void start() {
        m_lastNanos = System.nanoTime();
    }

    /** Stamp the duration since the last mark/start as section index {@code idx}. */
    public void mark(int idx) {
        long now = System.nanoTime();
        m_loggers[idx].accept((now - m_lastNanos) * 1e-6);
        m_lastNanos = now;
    }

    /**
     * Build a profiler keyed by name. Returns the profiler and a parallel list of
     * names so callers can refer to sections by name when wiring up.
     */
    public static final class Builder {
        private final String m_tab;
        private final List<String> m_sections = new ArrayList<>();

        public Builder(String tab) { m_tab = tab; }

        public Builder add(String section) { m_sections.add(section); return this; }

        public LoopProfiler build() {
            return new LoopProfiler(m_tab, m_sections.toArray(String[]::new));
        }

        public int indexOf(String section) { return m_sections.indexOf(section); }
    }
}
