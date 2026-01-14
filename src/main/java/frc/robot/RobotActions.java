package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.DriveToPosePID;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwervePayload;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.util.PoseUtil;
import frc.util.objective.ObjectiveIO;
import frc.util.objective.ObjectiveTracker;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachine.StateWithPayload;
import frc.util.statemachine.StateMachineState;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Contains the autonomous (and teleop) routines for the robot.
 */
public class RobotActions {

    /**
     * Locations of the robot to interact with the field elements.
     * Pose is relative to the blue alliance.
     */
    public enum FieldLocations {
        /**
         * The center of the hub.
         */
        // TODO: refine measurement
        HUB(new Pose2d(Inches.of(158.6 + (47.0 / 2.0)), Meters.of(4), Rotation2d.kZero));

        private final Pose2d blueAlliancePose;
        private final Pose2d redAlliancePose;

        /**
         * @return The pose of the location. Automatically flips if necessary.
         * @param shouldFlip - Whether or not to flip the pose. Defaults to true.
         * Also takes into account the current alliance color.
         */
        public Pose2d getPose(boolean shouldFlip) {
            return (shouldFlip && PoseUtil.shouldFlip()) ? redAlliancePose : blueAlliancePose;
        }

        public Pose2d getPose() {
            return getPose(true);
        }

        private FieldLocations(Pose2d pose) {
            this.blueAlliancePose = pose;
            this.redAlliancePose = FlippingUtil.flipFieldPose(pose);
        }
    }

    /**
     * Payload to point (auto aim) to the hub.
     */
    public static final SwervePayload POINT_TO_HUB_PAYLOAD = new SwervePayload(
        FieldLocations.HUB::getPose,
        () -> SwervePayload.RotationMode.ONLY_ROTATE_TO_POSE_NO_DRIVE_TO_POSE,
        FieldLocations.HUB::getPose
    );

    /**
     * List of global states for the robot.
     */
    public enum GlobalState {
        IDLE,

        INTAKE_CORAL_FROM_STATION,

        SCORE_CORAL,

        /**
         * Climbing.
         */
        CLIMBING,
    }

    /**
     * Payload for global states.
     * Specific states should extend this class to add more data.
     */
    public sealed interface GlobalPayload permits ScoreCoralPayload, IntakeCoralPayload, IdlePayload {}

    /**
     * Payload for scoring coral.
     * @param targetReefLocation - The location of the reef to score on.
     * @param targetLevel - The level to score on.
     */
    public record ScoreCoralPayload(FieldLocations targetReefLocation, int targetLevel) implements GlobalPayload {}

    /**
     * Payload for intaking coral.
     * @param targetCoralStation - The coral station to intake from.
     */
    public record IntakeCoralPayload(FieldLocations targetCoralStation) implements GlobalPayload {}

    /**
     * Payload for idle state.
     */
    public record IdlePayload() implements GlobalPayload {}

    public final StateMachine<GlobalState, GlobalPayload> globalStateMachine = new StateMachine<
        GlobalState,
        GlobalPayload
    >(GlobalState.class, "Global")
        .withDefaultState(new StateMachineState<>(GlobalState.IDLE, "Idle"))
        .withState(new StateMachineState<>(GlobalState.INTAKE_CORAL_FROM_STATION, "IntakeCoralStation"))
        .withState(new StateMachineState<>(GlobalState.SCORE_CORAL, "ScoreCoral"))
        .withState(new StateMachineState<>(GlobalState.CLIMBING, "Endgame"));

    public final ObjectiveTracker objectiveTracker;

    // References to subsystems
    // Public because of use in configuring controls
    public final Swerve swerveSubsystem;
    public final Vision visionSubsystem;

    /**
     * Creates a new AutoRoutines object given required subsystems.
     */
    public RobotActions(Swerve swerveSubsystem, Vision visionSubsystem, ObjectiveIO objectiveIO) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;

        this.objectiveTracker = new ObjectiveTracker(this, objectiveIO);
        // test
        // globalStateMachine.onStateChange(
        //     GlobalState.INTAKE_CORAL_FROM_STATION,
        //     IntakeCoralPayload.class,
        //     (previousState, payload) -> {
        //         // debug
        //         System.out.println("Switched to INTAKE_CORAL_FROM_STATION state with payload:");
        //         System.out.println("Target Coral Station: " + payload.targetCoralStation());

        //         swerveSubsystem.stateMachine.scheduleStateChange(
        //             new StateWithPayload<>(
        //                 Swerve.SwerveState.DRIVE_TO_POSE_PATHFINDING,
        //                 payload.targetCoralStation()::getPose
        //             )
        //         );
        //     }
        // );

        // globalStateMachine.onStateChange(GlobalState.SCORE_CORAL, ScoreCoralPayload.class, (previousState, payload) -> {
        //     // debug
        //     System.out.println("Switched to SCORE_CORAL state with payload:");
        //     System.out.println("Target Branch: " + payload.targetReefLocation());
        //     System.out.println("Target Level: " + payload.targetLevel());

        //     swerveSubsystem.stateMachine.scheduleStateChange(
        //         new StateWithPayload<>(
        //             Swerve.SwerveState.DRIVE_TO_POSE_PATHFINDING,
        //             payload.targetReefLocation()::getPose
        //         )
        //     );
        // });
    }

    /**
     * @return A command to move forward for a given amount of time at a given speed.
     * @param vxMetersPerSecond - The speed to move forward at in meters per second.
     * @param timeSeconds - The amount of time to move forward for in seconds.
     */
    private Command driveForwardWithSpeedFor(double vxMetersPerSecond, double timeSeconds) {
        return Commands.deadline(
            Commands.waitSeconds(timeSeconds),
            Commands.run(
                () -> swerveSubsystem.runVelocityChassisSpeeds(new ChassisSpeeds(vxMetersPerSecond, 0, 0)),
                swerveSubsystem
            )
        );
    }

    /**
     * @return A command that can be used in auto to move forward for a set amount of time.
     * More reliable than pathfinding.
     */
    public Command actuallyMoveForward() {
        return driveForwardWithSpeedFor(1.75, 3);
    }

    public Command pushAnotherRobotForward() {
        return driveForwardWithSpeedFor(3, 4);
    }
}
