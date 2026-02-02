package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ControlConstants;
import frc.robot.RobotActions.FieldLocations;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwervePayload;
import frc.robot.subsystems.swerve.Swerve.SwerveState;
import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The teleop swerve command.
 */
public class TeleopSwerve {

    /**
     * The drive to pose command used for final alignment.
     */
    public final DriveToPosePID driveToPoseCommand;

    private Command pathFindToPoseCommand = null;

    /**
     * The swerve subsystem.
     */
    private final Swerve swerveSubsystem;

    /**
     * The translation input (x), as a double from 0-1.
     */
    private final DoubleSupplier translationSupplier;

    /**
     * The strafe input (y), as a double from 0-1.
     */
    private final DoubleSupplier strafeSupplier;

    /**
     * The rotation input, as a double from 0-1.
     */
    private final DoubleSupplier rotationSupplier;

    /**
     * Whether the swerve is robot centric.
     * Default is always `false`.
     */
    private final BooleanSupplier robotCentricSupplier;

    /**
     * The speed multiplier, as a double.
     * Default is `1`
     */
    private final DoubleSupplier speedDial;

    /**
     * Whether to log values of the drive. Default is `true`
     */
    private final boolean logValues;

    /**
     * Creates the TeleopSwerve command.
     * The parameters are members of this class.
     */
    public TeleopSwerve(
        Swerve swerveSubsystem,
        DoubleSupplier translationSupplier,
        DoubleSupplier strafeSupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier robotCentricSupplier,
        DoubleSupplier speedDial,
        boolean logValues
    ) {
        this.swerveSubsystem = swerveSubsystem;

        // Set the values from the constructor
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
        this.speedDial = speedDial;
        this.logValues = logValues;

        driveToPoseCommand = new DriveToPosePID(this.swerveSubsystem, swerveSubsystem.autoAim);

        logCurrentStates();

        // When we enter full driver control, cancel any pathfinding commands and set state to not driving to pose
        swerveSubsystem.stateMachine.onStateChange(SwerveState.FULL_DRIVER_CONTROL, () -> {
            if (pathFindToPoseCommand != null) {
                pathFindToPoseCommand.cancel();
                pathFindToPoseCommand = null;
            }
        });

        // Bindings
        // TODO: make this command run before inputs/outputs are calcuated; this has a 1 frame delay between input and output currently
        swerveSubsystem.stateMachine.whileState(SwerveState.FULL_DRIVER_CONTROL, this::driveFullDriverControl);
        swerveSubsystem.stateMachine.whileState(SwerveState.AUTO_AIM, this::driveFullDriverControl);
        swerveSubsystem.stateMachine.whileState(SwerveState.DRIVE_TO_POSE_PATHFINDING, this::handleInitialPathfinding);
        swerveSubsystem.stateMachine.whileState(SwerveState.DRIVE_TO_POSE_PID, this::handleFinalAlignment);
        swerveSubsystem.stateMachine.whileState(SwerveState.DRIVE_TO_POSE_AT_TARGET, this::handleAtTarget);
    }

    /**
     * Creates the TeleopSwerve command given a {@link ControlConstants.Drive} object.
     */
    public TeleopSwerve(Swerve swerveSubsystem, ControlConstants.Drive driveControls, boolean logValues) {
        this(
            swerveSubsystem,
            driveControls::getTranslationValue,
            driveControls::getStrafeValue,
            driveControls::getRotationValue,
            driveControls::isRobotCentric,
            driveControls::getSpeedMultiplier,
            logValues
        );
    }

    /**
     * @return The chassis speeds based on controller input, scaled to {@link SwerveConstants#MAX_SPEED} and {@link SwerveConstants#ANGULAR_VELOCITY_FOR_TELEOP}.
     */
    public ChassisSpeeds getChassisSpeedsFromControls() {
        // Get values from suppliers
        double translationInput = translationSupplier.getAsDouble();
        double strafeInput = strafeSupplier.getAsDouble();
        double rotationInput = rotationSupplier.getAsDouble();
        double speedMultiplier = speedDial.getAsDouble();

        // Apply deadband and scale to max speed
        double vxMetersPerSecond =
            MathUtil.applyDeadband(translationInput, SwerveConstants.DRIVE_STICK_DEADBAND) *
            speedMultiplier *
            SwerveConstants.MAX_SPEED.in(MetersPerSecond);
        double vyMetersPerSecond =
            MathUtil.applyDeadband(strafeInput, SwerveConstants.DRIVE_STICK_DEADBAND) *
            speedMultiplier *
            SwerveConstants.MAX_SPEED.in(MetersPerSecond);

        double omegaRadPerSecond =
            MathUtil.applyDeadband(rotationInput, SwerveConstants.DRIVE_STICK_DEADBAND) *
            speedMultiplier *
            SwerveConstants.ANGULAR_VELOCITY_FOR_TELEOP.in(RadiansPerSecond);

        return swerveSubsystem.getSpeedsFromTranslation(
            vxMetersPerSecond,
            vyMetersPerSecond,
            omegaRadPerSecond,
            !robotCentricSupplier.getAsBoolean()
        );
    }

    /**
     * @param previous - The previous chassis speeds.
     * @return Apply a nudge to the previous chassis speeds based on controller input.
     */
    private ChassisSpeeds applyInputNudge(ChassisSpeeds previous) {
        // Get the nudge from the controller (as field centric)
        ChassisSpeeds inputtedNudge = getChassisSpeedsFromControls();

        // Convert to Translation2d to rotate around
        Translation2d previousTranslation = new Translation2d(previous.vxMetersPerSecond, previous.vyMetersPerSecond);

        Rotation2d rotateBy =
            // Use atan2 directly to avoid "x and y components of Rotation2d are zero" error
            new Rotation2d(Math.atan2(inputtedNudge.vyMetersPerSecond, inputtedNudge.vxMetersPerSecond))
                // Get the difference between the two angles
                .minus(new Rotation2d(Math.atan2(previous.vyMetersPerSecond, previous.vxMetersPerSecond)))
                .times(
                    // Scale down by the nudge input multiplier (avoid overshooting)
                    SwerveConstants.NUDGE_TRANSLATION_INPUT_MULTIPLIER *
                    // Scale the nudge based on the magnitude of the nudge
                    (Math.hypot(inputtedNudge.vxMetersPerSecond, inputtedNudge.vyMetersPerSecond) /
                        SwerveConstants.MAX_SPEED.in(MetersPerSecond))
                );

        Logger.recordOutput("Swerve/NudgeRotateBy", rotateBy.getDegrees());

        previousTranslation = previousTranslation.rotateBy(rotateBy);

        return new ChassisSpeeds(
            previousTranslation.getX(),
            previousTranslation.getY(),
            // Also nudge omega
            previous.omegaRadiansPerSecond +
            (inputtedNudge.omegaRadiansPerSecond * SwerveConstants.NUDGE_ROTATION_INPUT_MULTIPLIER)
        );
    }

    private void driveFullDriverControl() {
        // Drive based on the raw controller inputs
        ChassisSpeeds desiredChassisSpeeds = getChassisSpeedsFromControls();
        swerveSubsystem.runVelocityChassisSpeeds(desiredChassisSpeeds);
    }

    /**
     * Logs the current states of the drive to pose command.
     */
    private void logCurrentStates() {
        Logger.recordOutput(
            swerveSubsystem.stateMachine.dashboardKey + "/Targets",
            driveToPoseCommand.getAtTargetsRecords()
        );
    }

    /**
     * Handles auto-aiming.
     */
    // private void handleAutoAim(Optional<SwervePayload> payloadOptional) {
    //     ChassisSpeeds desiredChassisSpeeds = getChassisSpeedsFromControls();

    //     if (!payloadOptional.isPresent()) {
    //         // No target, just drive normally
    //         swerveSubsystem.runVelocityChassisSpeeds(desiredChassisSpeeds);
    //         return;
    //     }

    //     // Override rotation with auto-aim
    //     Pose2d robotPose = swerveSubsystem.getPose();
    //     Pose2d targetPose = payloadOptional.get().poseToRotateToSupplier().get();

    //     // TODO: also update this in DriveToPosePID
    //     // TODO: If this is not updated, shooter will not set target speed correctly
    //     swerveSubsystem.autoAim.updateCalculatedResult(robotPose, targetPose, desiredChassisSpeeds);
    //     swerveSubsystem.autoAim.latestCalculationResult.log();

    //     desiredChassisSpeeds.omegaRadiansPerSecond = swerveSubsystem.autoAim.getRotationOutputRadiansPerSecond();

    //     swerveSubsystem.runVelocityChassisSpeeds(desiredChassisSpeeds);
    // }

    /**
     * Handles the initial pathfinding to the target pose.
     */
    private void handleInitialPathfinding(Optional<SwervePayload> payloadOptional) {
        if (
            // The pathfinding command is already running
            pathFindToPoseCommand != null ||
            // No target pose supplier
            payloadOptional.isEmpty()
        ) {
            // Skip
            return;
        }

        SwervePayload payload = payloadOptional.get();
        // Supplier<Pose2d> targetPoseSupplier = payload.poseSupplier();
        Pose2d targetPose = payload.poseSupplier().get().getReference();

        Logger.recordOutput(swerveSubsystem.stateMachine.dashboardKey + "/TargetPose", targetPose);

        // Initialize pathfinding to the target pose (if not already doing so)
        pathFindToPoseCommand = new PathfindingCommand(
            targetPose,
            SwerveConstants.pathConstraints,
            swerveSubsystem::getPose,
            swerveSubsystem::getChassisSpeeds,
            (ChassisSpeeds speeds, DriveFeedforwards feedforwards) ->
                swerveSubsystem.runVelocityChassisSpeeds(applyInputNudge(speeds)),
            SwerveConstants.PP_INITIAL_PID_CONTROLLER,
            SwerveConstants.getRobotConfig()
        )
            // End when we can switch to final alignment
            .raceWith(Commands.waitUntil(driveToPoseCommand.canSwitchToFinalAlignment))
            .andThen(
                Commands.runOnce(() -> {
                    swerveSubsystem.stateMachine.scheduleStateChange(SwerveState.DRIVE_TO_POSE_PID);
                    pathFindToPoseCommand = null;
                })
            );

        CommandScheduler.getInstance().schedule(pathFindToPoseCommand);
    }

    /**
     * Handles the final alignment to the target pose.
     */
    private void handleFinalAlignment(Optional<SwervePayload> payloadOptional) {
        driveToPoseCommand.setOptionalPayload(payloadOptional);

        // Run final alignment
        swerveSubsystem.runVelocityChassisSpeeds(applyInputNudge(driveToPoseCommand.getChassisSpeeds()));

        // If at target, switch state
        if (driveToPoseCommand.atTarget.getAsBoolean()) {
            swerveSubsystem.stateMachine.scheduleStateChange(SwerveState.DRIVE_TO_POSE_AT_TARGET);
        }

        logCurrentStates();
    }

    /**
     * Handles being at the target pose.
     */
    private void handleAtTarget(Optional<SwervePayload> payloadOptional) {
        driveToPoseCommand.setOptionalPayload(payloadOptional);

        // If the robot is ever not at the target, go back to final alignment
        if (!driveToPoseCommand.atTarget.getAsBoolean()) {
            swerveSubsystem.stateMachine.scheduleStateChange(SwerveState.DRIVE_TO_POSE_PID);

            // Run a frame of final alignment to avoid a pause for 1 frame
            swerveSubsystem.runVelocityChassisSpeeds(applyInputNudge(driveToPoseCommand.getChassisSpeeds()));
            logCurrentStates();

            return;
        }

        // TODO: change this to a more elegant solution
        // swerveSubsystem.stop();
        driveFullDriverControl();
    }
}
