package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;

/**
 * A command to aim the robot to a target pose.
 */
public class AimToTarget {

    /**
     * A record containing the rates of change of distance and angle to the target pose.
     * See https://youtu.be/N6ogT5DjGOk&t=3613s for more information.
     * @param deltaDistanceRate - The rate of change of distance to the target pose in meters per second. Positive values indicate the robot is moving away from the target, negative values indicate the robot is moving towards the target.
     * @param deltaThetaRate - The rate of change of angle to the target pose in radians per second. If a circle is drawn centered at the target pose to the robot pose, positive values indicate the robot is rotating counterclockwise around the target, negative values indicate clockwise rotation.
     */
    public record DeltaPoseRates(double deltaDistanceRate, double deltaThetaRate) {
        public static final DeltaPoseRates ZERO = new DeltaPoseRates(0.0, 0.0);
    }

    /**
     * Returns both dr/dt and dθ/dt. See {@link DeltaPoseRates} for definitions.
     * @param robotPose - The current robot pose.
     * @param targetPose - The target pose.
     * @param robotRelativeSpeeds - The robot-relative chassis speeds.
     * @return A {@link DeltaPoseRates} record containing dr/dt and dθ/dt.
     */
    public static DeltaPoseRates getDeltaPoseRates(
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds robotRelativeSpeeds
    ) {
        // Convert robot-relative speeds to field-relative speeds
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds,
            robotPose.getRotation()
        );

        Translation2d deltaTranslation = targetPose.getTranslation().minus(robotPose.getTranslation());
        double distanceToTarget = deltaTranslation.getNorm();

        // Check for zero distance to avoid division by zero
        if (distanceToTarget < 1e-6) {
            return DeltaPoseRates.ZERO;
        }

        Translation2d directionToTarget = deltaTranslation.div(distanceToTarget);
        Translation2d tangentialDirection = new Translation2d(-directionToTarget.getY(), directionToTarget.getX());

        double velocityTowardsTarget =
            fieldRelativeSpeeds.vxMetersPerSecond * directionToTarget.getX() +
            fieldRelativeSpeeds.vyMetersPerSecond * directionToTarget.getY();

        double tangentialVelocity =
            fieldRelativeSpeeds.vxMetersPerSecond * tangentialDirection.getX() +
            fieldRelativeSpeeds.vyMetersPerSecond * tangentialDirection.getY();

        double deltaDistanceRate = -velocityTowardsTarget;
        double deltaThetaRate = -tangentialVelocity / distanceToTarget;

        return new DeltaPoseRates(deltaDistanceRate, deltaThetaRate);
    }

    /**
     * A record containing the rotation target in radians and feedforward in radians per second.
     * @param rotationTargetRadians - The rotation target in radians.
     * @param rotationFeedforwardRadiansPerSecond - The rotation feedforward in radians per second.
     */
    public record RotateToTargetCalculationResult(
        double rotationTargetRadians,
        double rotationFeedforwardRadiansPerSecond
    ) {}

    /**
     * Gets the rotation target in radians to face the target pose from the robot pose. Only considers translation, not rotation.
     * @param robotPose - The current robot pose.
     * @param targetPose - The target pose.
     * @return The rotation target in radians.
     */
    public static RotateToTargetCalculationResult getRotationTargetRadians(
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds robotRelativeSpeeds
    ) {
        double dxMeters = targetPose.getX() - robotPose.getX();
        double dyMeters = targetPose.getY() - robotPose.getY();

        double targetAngleRadians = Math.atan2(dyMeters, dxMeters);

        // double feedforward =
        //     Math.atan((robotRelativeSpeeds.vyMetersPerSecond - dyMeters) / -dxMeters) - Math.atan(dyMeters / dxMeters);

        double feedforward = getDeltaPoseRates(robotPose, targetPose, robotRelativeSpeeds).deltaThetaRate();

        return new RotateToTargetCalculationResult(targetAngleRadians, feedforward);
    }

    private final PIDController rotationController = new PIDController(
        SwerveConstants.PP_ROTATION_PID.kP,
        SwerveConstants.PP_ROTATION_PID.kI,
        SwerveConstants.PP_ROTATION_PID.kD
    );

    public AimToTarget() {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(SwerveConstants.AUTO_AIM_ANGLE_TOLERANCE.in(Radians));
    }

    /**
     * Gets the rotation output in radians per second to face the target pose from the robot pose.
     * @param robotPose - The current robot pose.
     * @param targetPose - The target pose.
     * @param robotRelativeSpeeds - The robot relative speeds.
     * @return The rotation output in radians per second.
     */
    public double getRotationOutputRadiansPerSecond(
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds robotRelativeSpeeds
    ) {
        RotateToTargetCalculationResult calculationResult = getRotationTargetRadians(
            robotPose,
            targetPose,
            robotRelativeSpeeds
        );

        // Calculate the PID output
        double pidRotationOutput = rotationController.calculate(
            robotPose.getRotation().getRadians(),
            calculationResult.rotationTargetRadians()
        );

        double unclampedOutput = calculationResult.rotationFeedforwardRadiansPerSecond();

        // Return 0 if at setpoint
        if (!rotationController.atSetpoint()) {
            unclampedOutput += pidRotationOutput;
        }

        Logger.recordOutput("AimToTarget/RotationTargetRadians", calculationResult.rotationTargetRadians());
        Logger.recordOutput("AimToTarget/PIDRotationOutput", pidRotationOutput);
        Logger.recordOutput("AimToTarget/FFRotationOutput", calculationResult.rotationFeedforwardRadiansPerSecond());

        // Clamp the output to the max angular velocity
        return MathUtil.clamp(
            unclampedOutput,
            -SwerveConstants.MAX_AUTO_AIM_ROBOT_ANGULAR_VELOCITY.in(RadiansPerSecond),
            SwerveConstants.MAX_AUTO_AIM_ROBOT_ANGULAR_VELOCITY.in(RadiansPerSecond)
        );
    }
}
