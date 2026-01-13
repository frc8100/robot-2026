package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwervePayload;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.util.PoseUtil;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Uses a PID controller to drive the robot to a specified pose. Does not use path finding/obstacle avoidance.
 */
public class DriveToPosePID {

    /**
     * A record containing whether the robot is at each target.
     */
    public record IsAtTargets(
        boolean atTarget,
        boolean atPoseTranslationTarget,
        boolean atPoseRotationTarget,
        boolean atVelocityTarget
    ) {
        public static final IsAtTargets ALL_FALSE = new IsAtTargets(false, false, false, false);
    }

    /**
     * The PID controller shared by all instances of this command.
     * Note: If multiple instances of this command are used simultaneously with different targets, this may cause issues.
     */
    // public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
    //     SwerveConstants.PP_ENDING_TRANSLATION_PID,
    //     SwerveConstants.PP_ROTATION_PID
    // );

    private final PIDController rotationController = new PIDController(
        SwerveConstants.PP_ROTATION_PID.kP,
        SwerveConstants.PP_ROTATION_PID.kI,
        SwerveConstants.PP_ROTATION_PID.kD
    );

    public static final Autopilot autopilot = new Autopilot(SwerveConstants.autopilotProfile);

    // Final alignment only checks position so the tolerance is any (360 degrees)
    private static final Angle finalAlignmentAngleTolerance = Degrees.of(360);

    private final Swerve swerveSubsystem;
    private final AimToTarget aimToTarget;

    private SwervePayload currentPayload;

    /**
     * A supplier that provides the target pose to drive to.
     * Called once per command execution.
     */
    private Supplier<Pose2d> targetPoseSupplier;

    /**
     * A trigger that is true when the robot is at the target pose.
     */
    public final BooleanSupplier atTarget;

    public final BooleanSupplier atPoseTranslationTarget;
    public final BooleanSupplier atPoseRotationTarget;
    public final BooleanSupplier atVelocityTarget;

    /**
     * Whether the robot can switch to final alignment mode.
     */
    public final BooleanSupplier canSwitchToFinalAlignment;

    /**
     * The goal state for the drive controller.
     */
    // private final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();

    private Pose2d lastTargetPose = Pose2d.kZero;
    private APTarget autopilotTarget = new APTarget(lastTargetPose);

    /**
     * Whether the target pose has changed since the last execution.
     */
    private boolean hasPoseChanged = true;

    /**
     * Creates a new DriveToPose command.
     * @param swerveSubsystem - The swerve drive subsystem.
     */
    public DriveToPosePID(Swerve swerveSubsystem, AimToTarget aimToTarget) {
        this.swerveSubsystem = swerveSubsystem;
        this.aimToTarget = aimToTarget;
        this.targetPoseSupplier = swerveSubsystem::getPose;
        this.currentPayload = SwervePayload.fromPoseSupplierNoRotate(this.targetPoseSupplier);

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.reset();

        atTarget = () ->
            PoseUtil.isPosesAndVelocityNear(
                swerveSubsystem.getPose(),
                this.targetPoseSupplier.get(),
                swerveSubsystem.getVelocityMagnitude(),
                SwerveConstants.targetVelocity,
                SwerveConstants.positionTolerance,
                SwerveConstants.angleTolerance,
                SwerveConstants.speedTolerance
            );

        canSwitchToFinalAlignment = () ->
            PoseUtil.isNear(
                swerveSubsystem.getPose(),
                this.targetPoseSupplier.get(),
                // Start final alignment when within this distance plus a bit based on current speed
                0.3 + swerveSubsystem.getVelocityMagnitude().in(MetersPerSecond) * 0.3,
                finalAlignmentAngleTolerance.in(Radians)
            );

        // debug
        atPoseTranslationTarget = () ->
            PoseUtil.isPoseTranslationNear(
                swerveSubsystem.getPose(),
                this.targetPoseSupplier.get(),
                SwerveConstants.positionTolerance
            );
        atPoseRotationTarget = () ->
            PoseUtil.isPoseRotationNear(
                swerveSubsystem.getPose(),
                this.targetPoseSupplier.get(),
                SwerveConstants.angleTolerance
            );
        atVelocityTarget = () ->
            PoseUtil.isVelocityNear(
                swerveSubsystem.getVelocityMagnitude(),
                SwerveConstants.targetVelocity,
                SwerveConstants.speedTolerance
            );
    }

    /**
     * Checks if the target pose has changed since the last execution. Also updates the last target pose.
     * @return Whether the target pose has changed since the last execution.
     */
    private boolean hasPoseChanged() {
        Pose2d currentTargetPose = targetPoseSupplier.get();
        boolean changed = !currentTargetPose.equals(lastTargetPose);
        lastTargetPose = currentTargetPose;

        // Update the autopilot target if the pose has changed
        if (changed) {
            // TODO: add way to customize entry angle, etc.
            autopilotTarget = new APTarget(lastTargetPose);
        }

        return changed;
    }

    public IsAtTargets getAtTargetsRecords() {
        return new IsAtTargets(
            atTarget.getAsBoolean(),
            atPoseTranslationTarget.getAsBoolean(),
            atPoseRotationTarget.getAsBoolean(),
            atVelocityTarget.getAsBoolean()
        );
    }

    /**
     * Sets a new payload.
     * @param newPayload - The new payload.
     */
    public void setPayload(SwervePayload newPayload) {
        this.currentPayload = newPayload;
        this.targetPoseSupplier = newPayload.poseSupplier();
    }

    /**
     * Sets a new target pose supplier.
     * @param newPayload - The new target pose supplier.
     */
    public void setOptionalPayload(Optional<SwervePayload> newPayload) {
        if (newPayload.isPresent()) {
            setPayload(newPayload.get());
        }
    }

    /**
     * Calculates the chassis speeds needed to drive to the target pose.
     * @return The chassis speeds needed to drive to the target pose.
     */
    public ChassisSpeeds getChassisSpeeds() {
        boolean poseChanged = hasPoseChanged();
        boolean shouldRotateToTarget = currentPayload
            .shouldRotateToPoseSupplier()
            .get()
            .equals(SwervePayload.RotationMode.ROTATE_AND_DRIVE_TO_POSE);

        Pose2d currentPose = swerveSubsystem.getPose();

        // Calculate translation chassis speeds from autopilot
        APResult autopilotResult = autopilot.calculate(
            currentPose,
            swerveSubsystem.getChassisSpeeds(),
            autopilotTarget
        );

        // If the pose has changed, reset the rotation controller to avoid windup
        if (poseChanged && !shouldRotateToTarget) {
            rotationController.reset();
        }

        double rotationSpeed = shouldRotateToTarget
            ? aimToTarget.getRotationOutputRadiansPerSecond(
                currentPose,
                lastTargetPose,
                swerveSubsystem.getChassisSpeeds()
            )
            : rotationController.calculate(
                currentPose.getRotation().getRadians(),
                autopilotResult.targetAngle().getRadians()
            );

        ChassisSpeeds outputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            autopilotResult.vx().in(MetersPerSecond),
            autopilotResult.vy().in(MetersPerSecond),
            rotationSpeed,
            currentPose.getRotation()
        );

        Logger.recordOutput("AP/Target", autopilotTarget.getReference());

        return outputSpeeds;
    }
}
