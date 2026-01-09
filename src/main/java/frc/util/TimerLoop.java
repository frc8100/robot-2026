package frc.util;

import edu.wpi.first.util.WPIUtilJNI;

/**
 * Runs a loop based on a timer.
 * Has queueing capabilities.
 */
public class TimerLoop {

    /**
     * The frequency in frames/executions per second.
     */
    private double fps = 60.0;

    /**
     * The current time
     */
    private long currentTime = WPIUtilJNI.now();
}
