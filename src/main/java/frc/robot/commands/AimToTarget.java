package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * A command to aim the robot to a target pose.
 */
public class AimToTarget {

    /**
     * @param distanceToTargetMeters - The distance to the target in meters.
     * @return The setpoint tolerance radius in radians.
     */
    public static double getSetpointToleranceRadiusRadians(double distanceToTargetMeters) {
        // TODO: store as a constant somewhere
        return Math.atan2(FieldConstants.hubRadiusForShooting.in(Meters), distanceToTargetMeters) * 0.3;
    }

    /**
     * A record containing the rates of change of distance and angle to the target pose.
     * See https://youtu.be/N6ogT5DjGOk&t=3613s for more information.
     * @param rotationTargetRadians - The rotation target in radians to face the target pose.
     * @param distanceToTargetMeters - The distance to the target pose in meters.
     * @param radialVelocity - The rate of change of distance to the target pose in meters per second. Positive values indicate the robot is moving away from the target, negative values indicate the robot is moving towards the target.
     * @param tangentialVelocity - The tangential velocity of the robot relative to the target pose in meters per second. Positive values indicate the robot is moving counterclockwise around the target, negative values indicate clockwise movement.
     * @param deltaThetaRate - The rate of change of angle to the target pose in radians per second. If a circle is drawn centered at the target pose to the robot pose, positive values indicate the robot is rotating counterclockwise around the target, negative values indicate clockwise rotation. This is one component of the total angular velocity feedforward needed to face the target.
     */
    public record AimCalculationResult(
        double rotationTargetRadians,
        double distanceToTargetMeters,
        double radialVelocity,
        double tangentialVelocity,
        double deltaThetaRate
    ) {
        public static final AimCalculationResult ZERO = new AimCalculationResult(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Returns both dr/dt and dθ/dt. See {@link AimCalculationResult} for definitions.
     * @param robotPose - The current robot pose.
     * @param targetPose - The target pose.
     * @param desiredRobotRelativeSpeeds - The robot-relative chassis speeds.
     * @return A {@link AimCalculationResult} record containing dr/dt and dθ/dt.
     */
    public static AimCalculationResult calculateAim(
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds desiredRobotRelativeSpeeds
    ) {
        // Convert robot-relative speeds to field-relative speeds
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            desiredRobotRelativeSpeeds,
            robotPose.getRotation()
        );

        Translation2d deltaTranslation = targetPose.getTranslation().minus(robotPose.getTranslation());

        // Rotation target is the angle of the vector from robot to target
        // double targetAngleRadians = Math.atan2(deltaTranslation.getY(), deltaTranslation.getX());

        double distanceToTarget = deltaTranslation.getNorm();

        // Check for zero distance to avoid division by zero
        if (distanceToTarget < 1e-6) {
            return AimCalculationResult.ZERO;
        }

        // Calculate direction and tangential direction unit vectors
        Translation2d directionToTarget = deltaTranslation.div(distanceToTarget);
        Translation2d tangentialDirection = new Translation2d(-directionToTarget.getY(), directionToTarget.getX());

        // Multiply field-relative speeds by direction vectors to get radial and tangential velocities
        double radialVelocityTowardsTarget =
            fieldRelativeSpeeds.vxMetersPerSecond * directionToTarget.getX() +
            fieldRelativeSpeeds.vyMetersPerSecond * directionToTarget.getY();

        double tangentialVelocity =
            fieldRelativeSpeeds.vxMetersPerSecond * tangentialDirection.getX() +
            fieldRelativeSpeeds.vyMetersPerSecond * tangentialDirection.getY();

        double deltaDistanceRate = -radialVelocityTowardsTarget;
        double deltaThetaRate = -tangentialVelocity / distanceToTarget;

        // Calculate effect of robot velocity on angle to target
        // targetAngleRadians += Math.copySign(
        //     getSetpointToleranceRadiusRadians(distanceToTarget),
        //     tangentialVelocity
        // );

        double targetAngleRadians = Math.atan2(deltaTranslation.getY(), deltaTranslation.getX());

        return new AimCalculationResult(
            targetAngleRadians,
            distanceToTarget,
            deltaDistanceRate,
            -tangentialVelocity,
            deltaThetaRate
        );
    }

    private final PIDController rotationController = new PIDController(
        SwerveConstants.PP_ROTATION_PID.kP,
        SwerveConstants.PP_ROTATION_PID.kI,
        SwerveConstants.PP_ROTATION_PID.kD
    );

    public AimToTarget() {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Gets the rotation output in radians per second to face the target pose from the robot pose.
     * @param robotPose - The current robot pose.
     * @param targetPose - The target pose.
     * @param desiredRobotRelativeSpeeds - The robot relative speeds.
     * @return The rotation output in radians per second.
     */
    public double getRotationOutputRadiansPerSecond(
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds desiredRobotRelativeSpeeds
    ) {
        AimCalculationResult calculationResult = calculateAim(robotPose, targetPose, desiredRobotRelativeSpeeds);

        double setpointToleranceRadians = getSetpointToleranceRadiusRadians(
            robotPose.getTranslation().getDistance(targetPose.getTranslation())
        );

        // Calculate the PID output
        double pidRotationOutput =
            // If we are within the setpoint tolerance, do not apply PID output
            MathUtil.isNear(
                    robotPose.getRotation().getRadians(),
                    calculationResult.rotationTargetRadians(),
                    setpointToleranceRadians
                )
                ? 0.0
                : rotationController.calculate(
                    robotPose.getRotation().getRadians(),
                    calculationResult.rotationTargetRadians()
                );

        double unclampedOutput = pidRotationOutput + calculationResult.deltaThetaRate();

        Logger.recordOutput("AimToTarget/RotationTargetRadians", calculationResult);
        Logger.recordOutput("AimToTarget/PIDRotationOutput", pidRotationOutput);

        // Clamp the output to the max angular velocity
        return MathUtil.clamp(
            unclampedOutput,
            -SwerveConstants.MAX_AUTO_AIM_ROBOT_ANGULAR_VELOCITY.in(RadiansPerSecond),
            SwerveConstants.MAX_AUTO_AIM_ROBOT_ANGULAR_VELOCITY.in(RadiansPerSecond)
        );
    }
}
