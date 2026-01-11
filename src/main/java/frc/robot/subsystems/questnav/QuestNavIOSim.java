package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.Swerve;
import frc.util.VelocityNoiseGenerator;
import gg.questnav.questnav.PoseFrame;
import java.util.function.Supplier;

/**
 * Simulates the QuestNav IO by providing pose data from the swerve subsystem.
 * Adds noise to the pose data to simulate real-world inaccuracies.
 * Simulates a 50Hz update rate (lower than real QuestNav) by providing one pose frame per update call.
 */
public class QuestNavIOSim implements QuestNavIO {

    // Noise parameters
    private static final Distance TRANSLATION_NOISE_BASE = Centimeters.of(0.05);
    private static final Distance TRANSLATION_NOISE_PER_VELOCITY = Centimeters.of(0.075);

    private static final Angle ROTATION_NOISE_BASE = Degrees.of(0.01);
    private static final Angle ROTATION_NOISE_PER_VELOCITY = Degrees.of(0.01);

    private final VelocityNoiseGenerator.PoseVelocityNoiseGenerator poseNoiseGenerator =
        new VelocityNoiseGenerator.PoseVelocityNoiseGenerator(
            TRANSLATION_NOISE_BASE,
            TRANSLATION_NOISE_PER_VELOCITY,
            ROTATION_NOISE_BASE,
            ROTATION_NOISE_PER_VELOCITY
        );

    private final Supplier<Pose2d> simulatedPoseSupplier;
    private final Swerve swerveSubsystem;
    private int frameCounter = 1;

    public QuestNavIOSim(Swerve swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        simulatedPoseSupplier = swerveSubsystem::getActualPose;
    }

    /**
     * @return The simulated pose with added noise.
     */
    private Pose3d getSimulatedPose() {
        Pose2d pose2d = simulatedPoseSupplier.get();

        // Add some noise to the simulated pose
        Pose2d noisyPose2d = poseNoiseGenerator.applyNoise(pose2d, swerveSubsystem.getVelocityMagnitudeAsDouble());

        return new Pose3d(noisyPose2d);
    }

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.connected = true;
        inputs.isTracking = true;
        inputs.batteryPercent = 100;
        inputs.trackingLostCounter = 0;

        // Add one pose frame per update
        inputs.unreadPoseFrames = new PoseFrame[] {
            new PoseFrame(
                getSimulatedPose().transformBy(QuestNavSubsystem.ROBOT_TO_QUEST),
                Timer.getTimestamp(),
                Timer.getTimestamp(),
                frameCounter,
                inputs.isTracking
            ),
        };

        frameCounter++;
    }
}
