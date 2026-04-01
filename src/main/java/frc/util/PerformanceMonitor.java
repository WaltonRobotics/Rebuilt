package frc.util;

import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;

import com.sun.management.OperatingSystemMXBean;

import frc.util.WaltLogger.DoubleLogger;

public class PerformanceMonitor {
    private static final String kLogTab = "Perf";

    private final DoubleLogger log_loopTimeMs = WaltLogger.logDouble(kLogTab, "loopTimeMs");
    private final DoubleLogger log_cpuUsagePct = WaltLogger.logDouble(kLogTab, "cpuUsagePct");
    private final DoubleLogger log_processCpuPct = WaltLogger.logDouble(kLogTab, "processCpuPct");
    private final DoubleLogger log_heapUsedMB = WaltLogger.logDouble(kLogTab, "heapUsedMB");
    private final DoubleLogger log_heapMaxMB = WaltLogger.logDouble(kLogTab, "heapMaxMB");
    private final DoubleLogger log_heapCommittedMB = WaltLogger.logDouble(kLogTab, "heapCommittedMB");
    private final DoubleLogger log_nonHeapUsedMB = WaltLogger.logDouble(kLogTab, "nonHeapUsedMB");

    private long m_loopStartNanos;
    private final OperatingSystemMXBean m_osBean;
    private final MemoryMXBean m_memBean;

    private static final double BYTES_TO_MB = 1.0 / (1024.0 * 1024.0);

    public PerformanceMonitor() {
        m_osBean = (OperatingSystemMXBean) ManagementFactory.getOperatingSystemMXBean();
        m_memBean = ManagementFactory.getMemoryMXBean();
    }

    /** Call at the very start of robotPeriodic(). */
    public void loopStart() {
        m_loopStartNanos = System.nanoTime();
    }

    /** Call at the very end of robotPeriodic(). Logs loop time, CPU, and memory usage. */
    public void loopEnd() {
        long now = System.nanoTime();
        log_loopTimeMs.accept((now - m_loopStartNanos) * 1e-6);
        log_cpuUsagePct.accept(m_osBean.getCpuLoad() * 100.0);
        log_processCpuPct.accept(m_osBean.getProcessCpuLoad() * 100.0);

        MemoryUsage heap = m_memBean.getHeapMemoryUsage();
        log_heapUsedMB.accept(heap.getUsed() * BYTES_TO_MB);
        log_heapMaxMB.accept(heap.getMax() * BYTES_TO_MB);
        log_heapCommittedMB.accept(heap.getCommitted() * BYTES_TO_MB);
        log_nonHeapUsedMB.accept(m_memBean.getNonHeapMemoryUsage().getUsed() * BYTES_TO_MB);
    }
}
