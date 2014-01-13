package kMinSpanningTreeCPLEX;

import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;

public class Tools {

	public static long CPUtime() {
		ThreadMXBean threadMXBean = ManagementFactory.getThreadMXBean();
		return threadMXBean.getCurrentThreadCpuTime();
	}

}
