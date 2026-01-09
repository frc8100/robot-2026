package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the QuestNav subsystem.
 */
public interface QuestNavIO {
    @AutoLog
    public static class QuestNavIOInputs {

        /**
         * True if the QuestNav is connected.
         * See {@link QuestNav#isConnected}.
         */
        public boolean connected = false;

        /**
         * True if the QuestNav is currently tracking.
         * See {@link QuestNav#isTracking}.
         */
        public boolean isTracking = false;

        /**
         * Battery percentage from 0 to 100, or -1 if unknown.
         * See {@link QuestNav#getBatteryPercent}.
         */
        public int batteryPercent = -1;

        /**
         * Number of times tracking was lost or -1 if unknown.
         * See {@link QuestNav#getTrackingLostCounter}.
         */
        public int trackingLostCounter = -1;

        /**
         * See {@link QuestNav#getAllUnreadPoseFrames}.
         */
        public PoseFrame[] unreadPoseFrames = new PoseFrame[0];
    }

    /** Updates the inputs of the QuestNav IO. */
    public default void updateInputs(QuestNavIOInputs inputs) {}

    /**
     * Sets the current pose of the QuestNav system.
     * ! IMPORTANT: This does not have to be transformed by the robot-to-headset transform; it will be handled internally.
     * See {@link https://questnav.gg/docs/getting-started/robot-code/#setting-robot-pose}
     * @param pose - The pose to set the QuestNav system to.
     */
    public default void setPose(Pose3d pose) {}

    /**
     * Sets the current pose of the QuestNav system using a 2D pose.
     * See {@link #setPose(Pose3d)} for important notes.
     * @param pose - The pose to set the QuestNav system to.
     */
    public default void setPose(Pose2d pose) {
        setPose(new Pose3d(pose));
    }
}
