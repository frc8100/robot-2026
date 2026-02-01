package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import gg.questnav.questnav.PoseFrame;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for interfacing with the QuestNav.
 */
public class QuestNavSubsystem extends SubsystemBase {

    /**
     * A measurement point used for calculating the transform between the QuestNav and the robot's coordinate frame.
     * See {@link #getMeasureTransformCommand(Swerve, Time)}.
     */
    private static record MeasurementPoint(double x, double y, double z) {}

    public static final Time DEFAULT_TIME_PER_MEASUREMENT = Seconds.of(0.1);

    /**
     * The transform from the robot's center to the headset. Used for offsetting pose data.
     */
    // TODO: Measure and set this transform
    public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
        new Translation3d(0.2, 0.2, 0.3),
        // Rotational transform shouldn't matter because of setPose handling, so just leave as identity
        Rotation3d.kZero
    );
    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    /**
     * The battery percentage threshold to trigger a low battery alert.
     * At or below this percentage, a warning alert will be triggered.
     */
    private static final int LOW_BATTERY_THRESHOLD = 20;

    /**
     * Consumer to send pose data to.
     */
    private final VisionConsumer consumer;

    private final QuestNavIO io;
    private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

    // Alerts
    private final Alert notConnectedAlert = new Alert("QuestNav not connected.", Alert.AlertType.kWarning);
    private final Alert lowBatteryAlert = new Alert("QuestNav battery low.", Alert.AlertType.kWarning);

    /**
     * The last battery percentage reported. Used to detect changes in battery level and update alerts accordingly.
     */
    private int lastBatteryPercent = -1;

    /**
     * Whether to consume pose data from the QuestNav ({@link #consumer}).
     * If false, pose data will be ignored, although it is still calculated.
     */
    public boolean shouldConsumePoseData = true;

    /**
     * Creates a new QuestNavSubsystem.
     * @param consumer - The vision consumer to send pose data to.
     * @param io - The IO implementation to use (real or simulated).
     */
    public QuestNavSubsystem(VisionConsumer consumer, QuestNavIO io) {
        this.consumer = consumer;
        this.io = io;
    }

    /**
     * Sets the current pose using a 2D pose.
     * See {@link QuestNavIO#setPose(Pose3d)} for more details.
     */
    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    /**
     * Sets the current pose using a 3D pose.
     * See {@link QuestNavIO#setPose(Pose3d)} for more details.
     */
    public void setPose(Pose3d pose) {
        io.setPose(pose);
    }

    @Override
    public void periodic() {
        // Update the IO inputs
        io.updateInputs(inputs);
        Logger.processInputs("QuestNav", inputs);

        // Handle alerts
        notConnectedAlert.set(inputs.connected);
        if (inputs.batteryPercent >= 0 && inputs.batteryPercent <= LOW_BATTERY_THRESHOLD) {
            lowBatteryAlert.set(true);

            // Update alert only if battery percentage has changed
            if (inputs.batteryPercent != lastBatteryPercent) {
                lowBatteryAlert.setText("QuestNav battery low: " + inputs.batteryPercent + "% remaining.");

                lastBatteryPercent = inputs.batteryPercent;
            }
        } else {
            lowBatteryAlert.set(false);
        }

        if (!inputs.isTracking) {
            // Not currently tracking, so don't do anything
            return;
        }

        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = inputs.unreadPoseFrames;

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Get the pose of the Quest
            Pose3d questPose = questFrame.questPose3d();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();

            // Transform by the mount pose to get your robot pose
            Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

            // You can put some sort of filtering here if you would like!

            // Add the measurement to our estimator
            consumer.accept(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        }
    }

    /**
     * @return A command that measures the transform between the QuestNav and the robot's coordinate frame.
     * @param swerveSubsystem - The swerve drive subsystem to use for movement.
     */
    public Command getMeasureTransformCommand(Swerve swerveSubsystem) {
        List<MeasurementPoint> measuredPoints = new ArrayList<>();

        Debouncer debounce = new Debouncer(DEFAULT_TIME_PER_MEASUREMENT.in(Seconds), DebounceType.kRising);
        ChassisSpeeds speedsToRunAt = new ChassisSpeeds(0, 0, 1);

        return Commands.runOnce(() -> {
            // Reset poses
            swerveSubsystem.setPose(new Pose2d());
            swerveSubsystem.zeroYawOffset();
            io.setPose(new Pose2d());
        }).andThen(
            Commands.run(
                () -> {
                    // Spin in place
                    swerveSubsystem.runVelocityChassisSpeeds(speedsToRunAt);

                    if (!inputs.isTracking) {
                        return;
                    }

                    // Debounce to limit measurement rate
                    if (!debounce.calculate(true)) {
                        return;
                    }

                    // Take a measurement
                    PoseFrame[] frames = inputs.unreadPoseFrames;
                    for (PoseFrame frame : frames) {
                        Pose3d questPose = frame.questPose3d();
                        Transform3d measurement = questPose.minus(
                            new Pose3d(0.0, 0.0, 0.0, new Rotation3d(swerveSubsystem.getHeadingFromGyro()))
                        );

                        measuredPoints.add(
                            new MeasurementPoint(measurement.getX(), measurement.getY(), measurement.getZ())
                        );
                    }

                    // Reset debounce
                    debounce.calculate(false);
                },
                swerveSubsystem
            ).finallyDo(() -> {
                // Calculate average
                double sumX = 0;
                double sumY = 0;
                double sumZ = 0;

                for (MeasurementPoint point : measuredPoints) {
                    sumX += point.x;
                    sumY += point.y;
                    sumZ += point.z;
                }

                double avgX = sumX / measuredPoints.size();
                double avgY = sumY / measuredPoints.size();
                double avgZ = sumZ / measuredPoints.size();

                NumberFormat formatter = new DecimalFormat("#0.000");

                System.out.println(
                    "Estimated ROBOT_TO_QUEST transform from " +
                    measuredPoints.size() +
                    " measurements:" +
                    " new Transform3d(" +
                    formatter.format(avgX) +
                    ", " +
                    formatter.format(avgY) +
                    ", " +
                    formatter.format(avgZ) +
                    ", " +
                    "new Rotation3d()" +
                    ");"
                );
            })
        );
    }
}
