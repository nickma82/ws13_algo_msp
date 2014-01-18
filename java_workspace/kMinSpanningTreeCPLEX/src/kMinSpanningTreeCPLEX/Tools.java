package kMinSpanningTreeCPLEX;

import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;

public class Tools {

	public static long CPUtime() {
		ThreadMXBean threadMXBean = ManagementFactory.getThreadMXBean();
		return threadMXBean.getCurrentThreadCpuTime();
	}
	
	public static double nanosecondsToSeconds(double nano) {
		return (double) nano / 1000000000.0;
	}

}
