package frc.robot.subsystems.battery;

import org.littletonrobotics.junction.AutoLog;

public interface BatteryIO {
    @AutoLog
    public static class BatteryIOInputs {

        /**
         * The current in amps being drawn from each channel of the power distribution at this loop cycle.
         * The length of this array should be equal to {@link BatteryLogger#NUM_CHANNELS}.
         * Index 0 is port 0, etc.
         */
        public double[] channelCurrentsAmps = new double[BatteryLogger.NUM_CHANNELS];

        /**
         * The total current in amps being drawn from the battery at this loop cycle. This should be equal to the sum of {@link #channelCurrentsAmps}.
         */
        public double totalCurrentAmps = 0.0;

        /**
         * The current voltage of the battery at this loop cycle.
         */
        public double currentBatteryVoltage = 12.0;
    }

    public default void updateInputs(BatteryIOInputs inputs) {}
}
