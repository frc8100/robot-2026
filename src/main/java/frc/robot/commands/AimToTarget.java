package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * A command to aim the robot to a target pose.
 */
public class AimToTarget {

    public record ExitVelocityCalculationResult(double exitVelocityMetersPerSecond, double timeToTargetSeconds) {
        public static final ExitVelocityCalculationResult ZERO = new ExitVelocityCalculationResult(0.0, 0.0);

        public static ExitVelocityCalculationResult interpolate(
            ExitVelocityCalculationResult startValue,
            ExitVelocityCalculationResult endValue,
            double t
        ) {
            return new ExitVelocityCalculationResult(
                MathUtil.interpolate(startValue.exitVelocityMetersPerSecond, endValue.exitVelocityMetersPerSecond, t),
                MathUtil.interpolate(startValue.timeToTargetSeconds, endValue.timeToTargetSeconds, t)
            );
        }

        public static Interpolator<ExitVelocityCalculationResult> getInterpolator() {
            return ExitVelocityCalculationResult::interpolate;
        }
    }

    /**
     * A map from distance to target in meters to time of flight in seconds.
     */
    public static final InterpolatingTreeMap<Double, ExitVelocityCalculationResult> distanceToTimeOfFlightMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ExitVelocityCalculationResult.getInterpolator());

    static {
        // Generate the distance to time of flight map based on calculations
        // TODO: try empirical data later
        for (double distance = 1.0; distance <= 15.0; distance += 0.2) {
            ExitVelocityCalculationResult result = solveExitVelocity(distance, 0.0, 0.0);
            distanceToTimeOfFlightMap.put(distance, result);
        }
    }

    /**
     * Solves for the exit velocity needed to hit the target at the given distance, accounting for the radial velocity of the robot.
     * @param distanceMeters - The horizontal distance to the target in meters.
     * @param radialVelocityMetersPerSecond - The radial velocity of the robot towards the target in meters per second. Positive values indicate the robot is moving away from the target, negative values indicate the robot is moving towards the target.
     * @return An {@link ExitVelocityCalculationResult} containing the exit velocity in meters per second and the time to target in seconds.
     */
    public static ExitVelocityCalculationResult solveExitVelocity(
        double distanceMeters,
        double radialVelocityMetersPerSecond,
        double tangentialVelocityMetersPerSecond
    ) {
        // Precompute constants
        final double cosHorizontalComponent = ShooterConstants.exitAngle.getCos();
        final double sinVerticalComponent = ShooterConstants.exitAngle.getSin();
        final double targetHeightOffset =
            FieldConstants.hubTargetHeight.in(Meters) - ShooterConstants.transformFromRobotCenter.getZ();
        final double gMPS2 = ShooterConstants.g.in(MetersPerSecondPerSecond);

        // Binary search bounds
        double velocityLowerBound = 1.0;
        double velocityUpperBound = 30.0;

        double timeToTargetSeconds = 0.0;

        for (int i = 0; i < 15; i++) {
            // Midpoint velocity
            double midpointVelocity = 0.5 * (velocityLowerBound + velocityUpperBound);

            // Calculate horizontal speed towards target, accounting for radial velocity
            double horizontalSpeed = midpointVelocity * cosHorizontalComponent + radialVelocityMetersPerSecond;
            if (horizontalSpeed <= 0.0) {
                velocityLowerBound = midpointVelocity;
                continue;
            }

            // First-order time of flight
            timeToTargetSeconds = distanceMeters / horizontalSpeed;

            // Tangential displacement during flight
            double lateralDisplacement = tangentialVelocityMetersPerSecond * timeToTargetSeconds;

            // Effective horizontal distance
            double effectiveDistance = Math.hypot(distanceMeters, lateralDisplacement);

            // Recompute time with corrected distance
            timeToTargetSeconds = effectiveDistance / horizontalSpeed;

            double heightAtTarget =
                midpointVelocity * sinVerticalComponent * timeToTargetSeconds -
                // Subtract the effect of gravity
                (0.5 * (gMPS2 * timeToTargetSeconds * timeToTargetSeconds));

            // Adjust bounds based on whether the height at the target is above or below the target height
            if (heightAtTarget > targetHeightOffset) {
                velocityUpperBound = midpointVelocity;
            } else {
                velocityLowerBound = midpointVelocity;
            }
        }

        // Return the average of the bounds as the solution
        return new ExitVelocityCalculationResult(0.5 * (velocityLowerBound + velocityUpperBound), timeToTargetSeconds);
    }

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
     * @param targetFuelExitVelocityMetersPerSecond - The velocity that the shooter should aim to exit fuel at, in meters per second.
     */
    // public record AimCalculationResult(
    //     double rotationTargetRadians,
    //     double distanceToTargetMeters,
    //     double radialVelocity,
    //     double tangentialVelocity,
    //     double deltaThetaRate,
    //     double targetFuelExitVelocityMetersPerSecond
    // ) {
    //     public static final AimCalculationResult ZERO = new AimCalculationResult(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // }
    public static class AimCalculationMutable {

        protected Pose2d robotPose = Pose2d.kZero;
        protected Pose2d targetPose = Pose2d.kZero;

        protected final MutAngle rotationTarget = Radians.mutable(0.0);
        protected final MutDistance distanceToTarget = Meters.mutable(0.0);
        protected final MutAngularVelocity radialVelocity = RadiansPerSecond.mutable(0.0);
        protected final MutLinearVelocity tangentialVelocity = MetersPerSecond.mutable(0.0);
        protected final MutAngularVelocity deltaThetaRate = RadiansPerSecond.mutable(0.0);
        protected final MutLinearVelocity targetFuelExitVelocity = MetersPerSecond.mutable(0.0);
        protected final MutAngularVelocity totalAngularVelocityFF = RadiansPerSecond.mutable(0.0);

        protected AimCalculationMutable() {
            // Log once at construction
            log();
        }

        public void log() {
            Logger.recordOutput("AimToTarget/TargetPose", targetPose);

            Logger.recordOutput("AimToTarget/RotationTarget", rotationTarget);
            Logger.recordOutput("AimToTarget/DistanceToTarget", distanceToTarget);
            Logger.recordOutput("AimToTarget/RadialVelocity", radialVelocity);
            Logger.recordOutput("AimToTarget/TangentialVelocity", tangentialVelocity);
            Logger.recordOutput("AimToTarget/DeltaThetaRate", deltaThetaRate);
            Logger.recordOutput("AimToTarget/TargetFuelExitVelocity", targetFuelExitVelocity);
            Logger.recordOutput("AimToTarget/TotalAngularVelocityFF", totalAngularVelocityFF);
        }

        // Getters
        public Angle getRotationTarget() {
            return rotationTarget;
        }

        public Distance getDistanceToTarget() {
            return distanceToTarget;
        }

        public AngularVelocity getRadialVelocity() {
            return radialVelocity;
        }

        public LinearVelocity getTangentialVelocity() {
            return tangentialVelocity;
        }

        public AngularVelocity getDeltaThetaRate() {
            return deltaThetaRate;
        }

        public LinearVelocity getTargetFuelExitVelocity() {
            return targetFuelExitVelocity;
        }

        public AngularVelocity getTotalAngularVelocityFF() {
            return totalAngularVelocityFF;
        }
    }

    public final AimCalculationMutable latestCalculationResult = new AimCalculationMutable();

    private final PIDController rotationController = new PIDController(
        SwerveConstants.PP_ROTATION_PID.kP,
        SwerveConstants.PP_ROTATION_PID.kI,
        SwerveConstants.PP_ROTATION_PID.kD
    );

    public AimToTarget() {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns an {@link AimCalculationResult} containing the rates of change of distance and angle to the target pose.
     * @param robotPose - The current robot pose.
     * @param targetPose - The target pose.
     * @param desiredRobotRelativeSpeeds - The robot-relative chassis speeds.
     * @param actualRobotRelativeSpeeds - The actual robot-relative chassis speeds.
     * @return A {@link AimCalculationResult}.
     */
    public void updateCalculatedResult(
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds desiredRobotRelativeSpeeds,
        ChassisSpeeds actualRobotRelativeSpeeds
    ) {
        // Convert robot-relative speeds to field-relative speeds
        ChassisSpeeds actualFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            actualRobotRelativeSpeeds,
            robotPose.getRotation()
        );
        // ChassisSpeeds desiredFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        //     desiredRobotRelativeSpeeds,
        //     robotPose.getRotation()
        // );
        ChassisSpeeds desiredFieldRelativeSpeeds = actualFieldRelativeSpeeds;

        Pose2d shooterPose = robotPose.transformBy(ShooterConstants.transformFromRobotCenter2d);

        Translation2d deltaTranslation = targetPose.getTranslation().minus(shooterPose.getTranslation());
        double distanceToTarget = deltaTranslation.getNorm();

        // Check for zero distance to avoid division by zero
        if (distanceToTarget < 1e-6) {
            return;
        }

        // Account for imparted velocity by robot to offset
        ExitVelocityCalculationResult calculatedResult = ExitVelocityCalculationResult.ZERO;
        Translation2d lookaheadPoseAsTranslation = Translation2d.kZero;
        double lookaheadPoseToTargetDistance = distanceToTarget;

        for (int i = 0; i < 15; i++) {
            calculatedResult = distanceToTimeOfFlightMap.get(lookaheadPoseToTargetDistance);
            Translation2d offset = new Translation2d(
                desiredFieldRelativeSpeeds.vxMetersPerSecond,
                desiredFieldRelativeSpeeds.vyMetersPerSecond
            ).times(calculatedResult.timeToTargetSeconds);

            lookaheadPoseAsTranslation = shooterPose.getTranslation().plus(offset);
            lookaheadPoseToTargetDistance = targetPose.getTranslation().getDistance(lookaheadPoseAsTranslation);
        }

        // Calculate direction
        deltaTranslation = targetPose.getTranslation().minus(lookaheadPoseAsTranslation);
        double targetAngleRadians = Math.atan2(deltaTranslation.getY(), deltaTranslation.getX());

        // TODO: Calculate feedforward
        Pose2d futureRobotPose = robotPose.exp(actualFieldRelativeSpeeds.toTwist2d(0.02));

        // Update the latest calculation result
        latestCalculationResult.robotPose = robotPose;
        latestCalculationResult.targetPose = new Pose2d(lookaheadPoseAsTranslation, targetPose.getRotation());

        latestCalculationResult.rotationTarget.mut_replace(targetAngleRadians, Radians);
        latestCalculationResult.distanceToTarget.mut_replace(distanceToTarget, Meters);
        latestCalculationResult.targetFuelExitVelocity.mut_replace(
            calculatedResult.exitVelocityMetersPerSecond,
            MetersPerSecond
        );
        // latestCalculationResult.totalAngularVelocityFF.mut_replace(yawFF, RadiansPerSecond);

        // debug
        Logger.recordOutput("AimToTarget/TimeToTargetSeconds", calculatedResult.timeToTargetSeconds);
        Logger.recordOutput(
            "AimToTarget/LookaheadPose",
            new Pose2d(lookaheadPoseAsTranslation, new Rotation2d(latestCalculationResult.getRotationTarget()))
        );
    }

    /**
     * Gets the rotation output in radians per second to face the target pose from the robot pose.
     * Must be called after {@link #updateCalculatedResult}.
     * @return The rotation output in radians per second.
     */
    public double getRotationOutputRadiansPerSecond() {
        double setpointToleranceRadians = getSetpointToleranceRadiusRadians(
            latestCalculationResult.robotPose
                .getTranslation()
                .getDistance(latestCalculationResult.targetPose.getTranslation())
        );

        // Calculate the PID output
        double pidRotationOutput =
            // If we are within the setpoint tolerance, do not apply PID output
            MathUtil.isNear(
                    latestCalculationResult.robotPose.getRotation().getRadians(),
                    latestCalculationResult.getRotationTarget().in(Radians),
                    setpointToleranceRadians
                )
                ? 0.0
                : rotationController.calculate(
                    latestCalculationResult.robotPose.getRotation().getRadians(),
                    latestCalculationResult.getRotationTarget().in(Radians)
                );

        double unclampedOutput =
            pidRotationOutput + latestCalculationResult.getTotalAngularVelocityFF().in(RadiansPerSecond);

        Logger.recordOutput("AimToTarget/PIDRotationOutput", pidRotationOutput);

        // Clamp the output to the max angular velocity
        return MathUtil.clamp(
            unclampedOutput,
            -SwerveConstants.MAX_AUTO_AIM_ROBOT_ANGULAR_VELOCITY.in(RadiansPerSecond),
            SwerveConstants.MAX_AUTO_AIM_ROBOT_ANGULAR_VELOCITY.in(RadiansPerSecond)
        );
    }
}
