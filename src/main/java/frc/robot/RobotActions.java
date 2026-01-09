package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPosePID;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.CriticalLevel;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.robot.subsystems.superstructure.claw.ClawConstants.IntakeOuttakeDirection;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.util.PoseUtil;
import frc.util.objective.ObjectiveTracker;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachine.StateWithPayload;
import frc.util.statemachine.StateMachineState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Contains the autonomous (and teleop) routines for the robot.
 */
public class RobotActions {

    /**
     * The types of field locations.
     */
    public enum FieldLocationType {
        LEFT_REEF,
        RIGHT_REEF,
        CORAL_STATION,
        PROCESSOR,
    }

    /**
     * Locations of the robot to interact with the field elements.
     * Pose is relative to the blue alliance.
     */
    public enum FieldLocations {
        REEF_1L(new Pose2d(3.74, 5, Rotation2d.fromDegrees(-60)), FieldLocationType.LEFT_REEF, 19, 6),
        REEF_1R(new Pose2d(4, 5.15, Rotation2d.fromDegrees(-60)), FieldLocationType.RIGHT_REEF, 19, 6),

        REEF_2L(new Pose2d(4.95, 5.16, Rotation2d.fromDegrees(-120)), FieldLocationType.LEFT_REEF, 20, 11),
        REEF_2R(new Pose2d(5.22, 5, Rotation2d.fromDegrees(-120)), FieldLocationType.RIGHT_REEF, 20, 11),

        REEF_3L(new Pose2d(5.7, 4.2, Rotation2d.fromDegrees(180)), FieldLocationType.LEFT_REEF, 21, 10),
        REEF_3R(new Pose2d(5.7, 3.87, Rotation2d.fromDegrees(180)), FieldLocationType.RIGHT_REEF, 21, 10),

        REEF_4L(new Pose2d(5.24, 3.06, Rotation2d.fromDegrees(120)), FieldLocationType.LEFT_REEF, 22, 9),
        REEF_4R(new Pose2d(4.96, 2.9, Rotation2d.fromDegrees(120)), FieldLocationType.RIGHT_REEF, 22, 9),

        REEF_5L(new Pose2d(4.03, 2.89, Rotation2d.fromDegrees(60)), FieldLocationType.LEFT_REEF, 17, 8),
        REEF_5R(new Pose2d(3.75, 3.06, Rotation2d.fromDegrees(60)), FieldLocationType.RIGHT_REEF, 17, 8),

        REEF_6L(new Pose2d(3.28, 3.85, Rotation2d.fromDegrees(0)), FieldLocationType.LEFT_REEF, 18, 7),
        REEF_6R(new Pose2d(3.28, 4.17, Rotation2d.fromDegrees(0)), FieldLocationType.RIGHT_REEF, 18, 7),

        CORAL_STATION_1(new Pose2d(1.1, 1.1, Rotation2d.fromDegrees(55)), FieldLocationType.CORAL_STATION, 12, 2),
        CORAL_STATION_2(new Pose2d(1.1, 6.8, Rotation2d.fromDegrees(-55)), FieldLocationType.CORAL_STATION, 13, 1),

        PROCESSOR(new Pose2d(6.732, 0.652, Rotation2d.fromDegrees(-90)), FieldLocationType.PROCESSOR, 16, 3);

        // A list of all the left reef poses and right reef poses, for easy access
        private static final List<Pose2d> leftReefPoses;
        private static final List<Pose2d> rightReefPoses;
        private static final List<Pose2d> coralStationPoses;

        // Initialize the lists of reef poses
        static {
            leftReefPoses = List.of(
                REEF_1L.getPose(),
                REEF_2L.getPose(),
                REEF_3L.getPose(),
                REEF_4L.getPose(),
                REEF_5L.getPose(),
                REEF_6L.getPose()
            );

            rightReefPoses = List.of(
                REEF_1R.getPose(),
                REEF_2R.getPose(),
                REEF_3R.getPose(),
                REEF_4R.getPose(),
                REEF_5R.getPose(),
                REEF_6R.getPose()
            );

            coralStationPoses = List.of(CORAL_STATION_1.getPose(), CORAL_STATION_2.getPose());
        }

        public static List<Pose2d> getLeftReefPoses() {
            return leftReefPoses;
        }

        public static List<Pose2d> getRightReefPoses() {
            return rightReefPoses;
        }

        public static List<Pose2d> getCoralStationPoses() {
            return coralStationPoses;
        }

        private final Pose2d pose;
        private final FieldLocationType type;

        // TODO: AprilTag IDs
        public final int blueAprilTagId;
        public final int redAprilTagId;

        /**
         * @return The pose of the location. Automatically flips if necessary.
         * @param shouldFlip - Whether or not to flip the pose. Defaults to true.
         * Also takes into account the current alliance color.
         */
        public Pose2d getPose(boolean shouldFlip) {
            return (shouldFlip && PoseUtil.shouldFlip()) ? FlippingUtil.flipFieldPose(pose) : pose;
        }

        public Pose2d getPose() {
            return getPose(true);
        }

        /**
         * @return The type of the location.
         */
        public FieldLocationType getType() {
            return type;
        }

        private FieldLocations(Pose2d pose, FieldLocationType type, int blueAprilTagId, int redAprilTagId) {
            this.pose = pose;
            this.type = type;
            this.blueAprilTagId = blueAprilTagId;
            this.redAprilTagId = redAprilTagId;
        }
    }

    /**
     * @return The rotation of the robot at the reef based on the tag ID.
     * @param tagId - The tag ID of the reef.
     */
    public static Rotation2d getReefRotationOfTargetPoseBasedOnTagId(int tagId) {
        Rotation2d output = new Rotation2d();

        switch (tagId) {
            // Blue
            case 19:
                output = Rotation2d.fromDegrees(-60);
                break;
            case 20:
                output = Rotation2d.fromDegrees(-120);
                break;
            case 21:
                output = Rotation2d.fromDegrees(180);
                break;
            case 22:
                output = Rotation2d.fromDegrees(120);
                break;
            case 17:
                output = Rotation2d.fromDegrees(60);
                break;
            case 18:
                output = Rotation2d.fromDegrees(0);
                break;
            // Red (rotation - pi)
            case 6:
                output = Rotation2d.fromDegrees(-60.0 + 180.0);
                break;
            case 11:
                output = Rotation2d.fromDegrees(-120.0 + 180.0);
                break;
            case 10:
                output = Rotation2d.fromDegrees(180.0 + 180.0);
                break;
            case 9:
                output = Rotation2d.fromDegrees(120.0 + 180.0);
                break;
            case 8:
                output = Rotation2d.fromDegrees(60.0 + 180.0);
                break;
            case 7:
                output = Rotation2d.fromDegrees(0.0 + 180.0);
                break;
            default:
                output = Rotation2d.fromDegrees(0);
                break;
        }

        return output;
    }

    /**
     * @return The pose of the nearest reef to the robot's current location.
     */
    private static Pose2d getNearestLeftReefGivenPosition(Pose2d currentPose) {
        return currentPose.nearest(FieldLocations.getLeftReefPoses());
    }

    private static Pose2d getNearestRightReefGivenPosition(Pose2d currentPose) {
        return currentPose.nearest(FieldLocations.getRightReefPoses());
    }

    private static Pose2d getNearestCoralStationGivenPosition(Pose2d currentPose) {
        return currentPose.nearest(FieldLocations.getCoralStationPoses());
    }

    /**
     * List of global states for the robot.
     */
    public enum GlobalState {
        IDLE,

        INTAKE_CORAL_FROM_STATION,

        SCORE_CORAL,

        /**
         * Climbing. No scoring actions allowed.
         */
        ENDGAME,
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
    public record ScoreCoralPayload(FieldLocations targetReefLocation, SuperstructureConstants.Level targetLevel)
        implements GlobalPayload {}

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
        .withState(new StateMachineState<>(GlobalState.ENDGAME, "Endgame"));

    public final ObjectiveTracker objectiveTracker = new ObjectiveTracker(this);

    // References to subsystems
    // Public because of use in configuring controls
    public final Swerve swerveSubsystem;
    public final Elevator elevatorSubsystem;
    public final Claw clawSubsystem;
    public final Vision visionSubsystem;

    /**
     * Creates a new AutoRoutines object given required subsystems.
     */
    public RobotActions(
        Swerve swerveSubsystem,
        Elevator elevatorSubsystem,
        Claw clawSubsystem,
        Vision visionSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.visionSubsystem = visionSubsystem;

        // test
        globalStateMachine.onStateChange(
            GlobalState.INTAKE_CORAL_FROM_STATION,
            IntakeCoralPayload.class,
            (previousState, payload) -> {
                // debug
                System.out.println("Switched to INTAKE_CORAL_FROM_STATION state with payload:");
                System.out.println("Target Coral Station: " + payload.targetCoralStation());

                swerveSubsystem.stateMachine.scheduleStateChange(
                    new StateWithPayload<>(
                        Swerve.SwerveState.DRIVE_TO_POSE_PATHFINDING,
                        payload.targetCoralStation()::getPose
                    )
                );
            }
        );

        globalStateMachine.onStateChange(GlobalState.SCORE_CORAL, ScoreCoralPayload.class, (previousState, payload) -> {
            // debug
            System.out.println("Switched to SCORE_CORAL state with payload:");
            System.out.println("Target Branch: " + payload.targetReefLocation());
            System.out.println("Target Level: " + payload.targetLevel());

            swerveSubsystem.stateMachine.scheduleStateChange(
                new StateWithPayload<>(
                    Swerve.SwerveState.DRIVE_TO_POSE_PATHFINDING,
                    payload.targetReefLocation()::getPose
                )
            );
        });
    }

    public boolean isCoralIntakeComplete() {
        return clawSubsystem.isCoralInClaw();
    }

    /**
     * @return The pose of the nearest left reef to the robot's current location.
     */
    public Pose2d getNearestLeftReef() {
        return getNearestLeftReefGivenPosition(swerveSubsystem.getPose());
    }

    /**
     * @return The pose of the nearest right reef to the robot's current location.
     */
    public Pose2d getNearestRightReef() {
        return getNearestRightReefGivenPosition(swerveSubsystem.getPose());
    }

    public Pose2d getNearestCoralStation() {
        return getNearestCoralStationGivenPosition(swerveSubsystem.getPose());
    }

    /**
     * @return A command to set up the superstructure for a given level by moving the elevator and claw to the correct positions.
     */
    public DeferredCommand setUpSuperstructure(SuperstructureConstants.Level level) {
        // return clawSubsystem
        //     // Move claw out of way
        //     .getWaitForAngleCommand(ClawConstants.RotationPositions.CLAW_HOLDING_POSITION)
        //     // Raise elevator
        //     .andThen(elevatorSubsystem.getPositionCommandAndWait(level))
        //     // Move claw to level angle
        //     .andThen(clawSubsystem.getWaitForAngleCommand(level.getClawAngle()))
        //     // Stop when interrupted
        //     .handleInterrupt(() -> {
        //         elevatorSubsystem.io.resetSetpointToCurrentPosition();
        //         clawSubsystem.io.resetSetpointToCurrentPosition();
        //     });

        return new DeferredCommand(() -> setUpSuperstructureDeferred(level), Set.of(elevatorSubsystem, clawSubsystem));
    }

    private Command setUpSuperstructureDeferred(SuperstructureConstants.Level level) {
        return clawSubsystem
            // Move claw out of way
            .getWaitForAngleCommand(ClawConstants.RotationPositions.CLAW_HOLDING_POSITION)
            // Raise elevator
            .alongWith(elevatorSubsystem.getPositionCommandAndWaitNotNearer(level))
            // Move claw to level angle
            .andThen(clawSubsystem.getWaitForAngleCommand(level.getClawAngle()))
            // Stop when interrupted
            .handleInterrupt(() -> {
                elevatorSubsystem.io.resetSetpointToCurrentPosition();
                clawSubsystem.io.resetSetpointToCurrentPosition();
            });
    }

    private Command setUpSuperstructureDeferredOld(SuperstructureConstants.Level level) {
        Angle elevatorCurrentPosition = elevatorSubsystem.getAngularPosition();
        Angle elevatorTargetPosition = Radians.of(level.getElevatorRadian());

        ArrayList<CriticalLevel> elevatorCriticalPositions = new ArrayList<>();

        // Check if any critical levels are reached when moving the elevator from the current position to the target position
        for (CriticalLevel criticalLevel : SuperstructureConstants.CRITICAL_LEVELS) {
            // If the elevator is moving up and will reach a critical level
            if (
                elevatorCurrentPosition.lt(criticalLevel.getElevatorAngle()) &&
                elevatorTargetPosition.gte(criticalLevel.getElevatorAngle())
            ) {
                elevatorCriticalPositions.add(
                    // new CriticalLevelRaw(criticalLevel.getLowerElevatorRadian(), criticalLevel.getClawAngle())
                    criticalLevel
                );
            }

            // If the elevator is moving down and will reach a critical level
            if (
                elevatorCurrentPosition.gt(criticalLevel.getElevatorAngle()) &&
                elevatorTargetPosition.lte(criticalLevel.getElevatorAngle())
            ) {
                elevatorCriticalPositions.add(
                    // new CriticalLevelRaw(criticalLevel.getUpperElevatorRadian(), criticalLevel.getClawAngle())
                    criticalLevel
                );
            }
        }

        // test
        // Logger.recordOutput("TestElevator/ElevatorCurrentPosition", elevatorCurrentPosition);
        // Logger.recordOutput("TestElevator/ElevatorTargetPosition", elevatorTargetPosition);
        // Logger.recordOutput(
        //     "TestElevator/ElevatorCriticalPositions",
        //     elevatorCriticalPositions
        //         .stream()
        //         .mapToDouble((CriticalLevel c) -> c.getElevatorAngle().in(Radians))
        //         .toArray()
        // );
        // Logger.recordOutput("TestElevator/IsElevatorCriticalPositionsEmpty", elevatorCriticalPositions.isEmpty());

        // If there are no critical levels to wait for, just move the elevator
        if (elevatorCriticalPositions.isEmpty()) {
            return elevatorSubsystem
                .getPositionCommandAndWait(level)
                // Move claw to level angle
                .alongWith(clawSubsystem.getWaitForAngleCommand(level.getClawAngle()))
                // Stop when interrupted
                .handleInterrupt(() -> {
                    elevatorSubsystem.io.resetSetpointToCurrentPosition();
                    clawSubsystem.io.resetSetpointToCurrentPosition();
                });
        }

        // If there are critical levels to wait for, move the elevator to the each critical level
        SequentialCommandGroup command = new SequentialCommandGroup();

        // TODO: Only have one elevator move command in total and move the claw in parallel
        for (CriticalLevel criticalLevel : elevatorCriticalPositions) {
            command.addCommands(
                // Simultaneously move the elevator and claw to the critical level holding position
                elevatorSubsystem
                    .getPositionCommandAndWait(criticalLevel.getFirstElevatorRadian(elevatorCurrentPosition))
                    .alongWith(clawSubsystem.getWaitForAngleCommand(criticalLevel.getClawAngle())),
                // Then move the elevator over the critical level without moving the claw
                elevatorSubsystem.getPositionCommandAndWait(
                    criticalLevel.getSecondElevatorRadian(elevatorCurrentPosition)
                )
            );
        }

        // Move the elevator to the target level
        command.addCommands(
            elevatorSubsystem
                .getPositionCommandAndWait(level)
                // Move claw to level angle
                .alongWith(clawSubsystem.getWaitForAngleCommand(level.getClawAngle()))
        );

        // Stop when interrupted
        return command.handleInterrupt(() -> {
            elevatorSubsystem.io.resetSetpointToCurrentPosition();
            clawSubsystem.io.resetSetpointToCurrentPosition();
        });
    }

    /**
     * @return A command to do the elevator and claw movements for the intake of algae from L2.
     */
    public Command intakeAlgae(SuperstructureConstants.Level algaeLevel) {
        // Move superstructure up
        return setUpSuperstructure(algaeLevel)
            .andThen(
                new ParallelCommandGroup(
                    // Intake and hold algae
                    clawSubsystem.runIntakeOrOuttake(IntakeOuttakeDirection.BACK, ClawConstants.ALGAE_TIMEOUT_TIME),
                    // Raise elevator a bit
                    Commands.run(() -> elevatorSubsystem.runMotor(1))
                        .withTimeout(Seconds.of(0.5))
                        // Lower elevator
                        .andThen(
                            elevatorSubsystem.getPositionCommandAndWait(SuperstructureConstants.Level.INITIAL_POSITION)
                        )
                    // Move back
                    // .alongWith(driveForwardWithSpeedFor(-1.5, 0.4))
                )
            )
            // Stop when interrupted
            .handleInterrupt(() -> {
                elevatorSubsystem.io.resetSetpointToCurrentPosition();
                clawSubsystem.io.resetSetpointToCurrentPosition();
            });
    }

    /**
     * @return A command to do only the claw movements for L4.
     */
    public Command doClawMovementsForL4() {
        return clawSubsystem
            .getWaitForAngleCommand(ClawConstants.RotationPositions.CLAW_L4_SCORING_POSITION)
            // TODO: Claw will not register that coral is not in claw for L4 because it is too close to the branch
            .andThen(clawSubsystem.runOuttakeUntilCoralIsNotInClaw())
            .andThen(clawSubsystem.getWaitForAngleCommand(SuperstructureConstants.Level.L4.getClawAngle()))
            // Stop when interrupted
            .handleInterrupt(() -> {
                clawSubsystem.io.resetSetpointToCurrentPosition();
                clawSubsystem.io.runOuttake(0);
            });
        //
    }

    /**
     * @return A command to pathfind to a given location.
     * @param location - The location to pathfind to. See {@link FieldLocations} for possible locations.
     * @param shouldAlignToReef - Whether or not to align to the reef tag after pathfinding.
     */
    public Command pathFindToLocation(FieldLocations location) {
        return pathFindToLocation(location::getPose);
    }

    /**
     * @return A command to pathfind to a given pose using pathplanner.
     * @param pose - The pose to pathfind to. Automatically flips if necessary.
     */
    public Command pathFindToLocation(Supplier<Pose2d> pose) {
        // return AutoBuilder.pathfindToPose(pose, SwerveConfig.pathConstraints);
        // return new DeferredCommand(() -> new DriveToPose(swerveSubsystem, () -> pose), Set.of(swerveSubsystem));
        return new DeferredCommand(
            () ->
                new PathfindingCommand(
                    pose.get(),
                    SwerveConstants.pathConstraints,
                    swerveSubsystem::getPose,
                    swerveSubsystem::getChassisSpeeds,
                    (speeds, feedforwards) -> swerveSubsystem.runVelocityChassisSpeeds(speeds),
                    DriveToPosePID.driveController,
                    SwerveConstants.getRobotConfig()
                ),
            Set.of(swerveSubsystem)
        );
        // return new DeferredCommand(
        //     () ->
        //         // Use PathPlanner path finding for initial pathfinding
        //         AutoBuilder.pathfindToPose(pose.get(), SwerveConfig.pathConstraints)
        //             .raceWith(
        //                 Commands.waitUntil(() ->
        //                     // Path find until close enough for final alignment
        //                     PoseUtil.isNear(
        //                         swerveSubsystem.getPose(),
        //                         pose.get(),
        //                         // Start final alignment when within this distance plus a bit based on current speed
        //                         Meters.of(0.4 + swerveSubsystem.getVelocityMagnitude().in(MetersPerSecond) * 0.4),
        //                         Degrees.of(360)
        //                     )
        //                 )
        //             )
        //             // Use DriveToPose for final alignment
        //             .andThen(new DriveToPose(swerveSubsystem, pose)),
        //     Set.of(swerveSubsystem)
        // );

        // return Commands.none();
    }

    /**
     * @return A command to continuously pathfind to a given location.
     * Useful for teleop, as the robot will always be trying to go to the location.
     * @param location - The location to pathfind to. See {@link FieldLocations} for possible locations.
     */
    public Command continuouslyPathFindToLocation(Supplier<Pose2d> pose) {
        return new DeferredCommand(
            () ->
                pathFindToLocation(pose)
                    // .andThen(Commands.waitSeconds(0.1))
                    .withTimeout(0.1)
                    .andThen(continuouslyPathFindToLocation(pose)),
            Set.of(swerveSubsystem)
        );
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

    public Command moveForwardAndL1() {
        return driveForwardWithSpeedFor(0.75, 2.25)
            .alongWith(setUpSuperstructure(SuperstructureConstants.Level.L1_AUTO))
            .andThen(clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.BACK));
    }

    public Command launchAlgae() {
        return new ParallelRaceGroup(
            // Continuously hold until done with elevator movements
            clawSubsystem.runIntakeOrOuttake(IntakeOuttakeDirection.BACK, ClawConstants.ALGAE_TIMEOUT_TIME),
            // Raise elevator
            setUpSuperstructure(SuperstructureConstants.Level.ALGAE_HOLD_NET)
        ).andThen(
            // Launch
            // clawSubsystem
            //     .runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE)
            //     .alongWith(
            //         clawSubsystem.getWaitForAngleCommand(ClawConstants.RotationPositions.CLAW_ALGAE_THROW_POSITION)
            //     )
            clawSubsystem
                .getWaitForAngleCommand(ClawConstants.RotationPositions.CLAW_ALGAE_THROW_POSITION)
                .alongWith(
                    Commands.waitTime(Seconds.of(0.3)).andThen(
                        clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE)
                    )
                )
        );
    }

    /**
     * @return Align with the reef, and score on the level.
     */
    // public Command alignAndScore(SuperstructureConstants.Level levelToScoreOn) {
    //     return new PhotonVisionAlign(false, swerveSubsystem, visionSubsystem)
    //         .alongWith(setUpSuperstructure(levelToScoreOn))
    //         .andThen(clawSubsystem.runOuttakeUntilCoralIsNotInClaw());
    // }

    /**
     * @return A command to get coral and go to all reefs, repeating the process.
     */
    public Command getCoralAndGoToAllReefsTest() {
        return new SequentialCommandGroup(
            // Get coral from station 1
            pathFindToLocation(FieldLocations.CORAL_STATION_1),
            // Run intake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.BACK).withTimeout(Seconds.of(1)),
            // Simultaneously set up superstructure and pathfind to reef
            new ParallelCommandGroup(
                pathFindToLocation(FieldLocations.REEF_4L),
                Commands.waitSeconds(1.75).andThen(setUpSuperstructure(SuperstructureConstants.Level.L4))
            ),
            // Run outtake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE).withTimeout(Seconds.of(1)),
            // Reset superstructure
            // Simultaneously set up superstructure and pathfind to coral station
            new ParallelCommandGroup(
                pathFindToLocation(FieldLocations.CORAL_STATION_1),
                setUpSuperstructure(SuperstructureConstants.Level.INITIAL_POSITION)
            ),
            // Run intake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.BACK).withTimeout(Seconds.of(1)),
            // Simultaneously set up superstructure and pathfind to reef
            new ParallelCommandGroup(
                pathFindToLocation(FieldLocations.REEF_4R),
                Commands.waitSeconds(1.75).andThen(setUpSuperstructure(SuperstructureConstants.Level.L4))
            ),
            // Run outtake
            clawSubsystem.runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE).withTimeout(Seconds.of(1)),
            // Reset superstructure
            setUpSuperstructure(SuperstructureConstants.Level.INITIAL_POSITION)
        );
    }
}
