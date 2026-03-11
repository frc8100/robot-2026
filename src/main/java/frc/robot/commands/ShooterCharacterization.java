package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A routine to characterize the shooter by shooting balls at various distances and velocities, and recording the time it takes for the ball to reach the target and whether the shot was successful.
 *
 * Routine: (video record the process to analyze time to target in slow mo later)
 * 1. Set up robot at a known distance from the target (e.g. 1 feet).
 * 2. Enable the robot. This command will wind up and shoot the ball at the starting angular velocity.
 * 3. Once the ball is shot, disable the robot.
 * 4. Once the ball reaches the target, check a box whether the shot was successful (on the dashboard).
 * 5. Repeat steps 2-4 for the number of shots per velocity; this command will then increase the velocity by the velocity step and repeat steps 2-4 until it is past a speed where it once could have reached the target but can no longer reach the target.
 * 6. Move the robot back by a distance step.
 */
public class ShooterCharacterization extends Command {

    public static final String DASHBOARD_PREFIX = "ShooterCharacterization";

    // TODO: tune this
    public static final AngularVelocity STARTING_ANGULAR_VELOCITY = RadiansPerSecond.of(120);
    public static final AngularVelocity ENDING_ANGULAR_VELOCITY = RadiansPerSecond.of(900);
    public static final AngularVelocity VELOCITY_STEP = RadiansPerSecond.of(30);

    public static final AngularVelocity VELOCITY_STEP_WHEN_NEW_DISTANCE = RadiansPerSecond.of(-120);

    public static final AngularVelocity ANGULAR_VELOCITY_TOLERANCE = RadiansPerSecond.of(3.0);

    public static final int NUMBER_OF_SHOTS_PER_VELOCITY = 2;

    public static final Distance STARTING_DISTANCE = Inches.of(12);
    public static final Distance DISTANCE_STEP = Inches.of(6);

    /**
     * A data point representing a single shot taken during the characterization process,
     * Does not include time until target, as that will be analyzed later from the video recordings of the shots.
     */
    public record TestingShooterDataPoint(AngularVelocity shooterAngularVelocity, boolean successfulShot) {
        @Override
        public final String toString() {
            return (
                "TestingShooterDataPoint{" +
                "shooterAngularVelocity=" +
                shooterAngularVelocity.in(RPM) +
                " RPM, successfulShot=" +
                successfulShot +
                '}'
            );
        }
    }

    /**
     * A map of distance to the data points taken at that distance.
     */
    public static final Map<Double, TestingShooterDataPoint> shooterDataPoints = new TreeMap<>();

    public final Shooter shooterSubsystem;

    public final LoggedNetworkNumber distanceOverride = new LoggedNetworkNumber(
        DASHBOARD_PREFIX + "/DistanceOverrideInches",
        0.0
    );
    public final LoggedNetworkBoolean successfulShotInput = new LoggedNetworkBoolean(
        DASHBOARD_PREFIX + "/SuccessfulShotInput",
        false
    );
    public final LoggedNetworkNumber speedOverride = new LoggedNetworkNumber(
        DASHBOARD_PREFIX + "/SpeedOverrideRadPerSec",
        STARTING_ANGULAR_VELOCITY.in(RadiansPerSecond)
    );

    // Current state
    public final MutAngularVelocity currentAngularVelocityTarget = STARTING_ANGULAR_VELOCITY.mutableCopy();
    public final MutDistance currentDistance = STARTING_DISTANCE.mutableCopy();
    public int shotsTakenAtCurrentVelocity = 0;

    /**
     * Whether we have hit the target at the current distance. This is used to determine when to stop the characterization process, as once we have exceeded the maximum velocity that can reach the target, we can stop.
     */
    public boolean hasHitTargetAtCurrentDistance = false;

    public ShooterCharacterization(Shooter shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    private void incrementDistanceAndResetVelocity() {
        currentDistance.mut_plus(DISTANCE_STEP);

        // currentAngularVelocityTarget.mut_replace(STARTING_ANGULAR_VELOCITY);
        // Decrease angular velocity a bit (for faster)
        currentAngularVelocityTarget.mut_plus(VELOCITY_STEP_WHEN_NEW_DISTANCE);

        hasHitTargetAtCurrentDistance = false;

        System.out.println(
            "Moving to next distance: " +
            currentDistance.in(Inches) +
            " inches. Resetting velocity to: " +
            currentAngularVelocityTarget.in(RPM) +
            " RPM."
        );
    }

    private void logState() {
        Logger.recordOutput(DASHBOARD_PREFIX + "/CurrentDistanceInches", currentDistance.in(Inches));
        Logger.recordOutput(DASHBOARD_PREFIX + "/CurrentAngularVelocityRPM", currentAngularVelocityTarget.in(RPM));
        Logger.recordOutput(DASHBOARD_PREFIX + "/ShotsTakenAtCurrentVelocity", shotsTakenAtCurrentVelocity);

        // TODO: move this to shooter subsystem
        Logger.recordOutput(
            DASHBOARD_PREFIX + "/VelocityAtTarget",
            shooterSubsystem
                .getCurrentMotorAngularVelocity()
                .isNear(currentAngularVelocityTarget, ANGULAR_VELOCITY_TOLERANCE)
        );
    }

    /**
     * Handles logic for when the robot is enabled.
     */
    @Override
    public void initialize() {
        // Read the successful shot input from the dashboard
        boolean wasPreviouslySuccessful = successfulShotInput.get();
        TestingShooterDataPoint dataPoint = new TestingShooterDataPoint(
            currentAngularVelocityTarget.copy(),
            wasPreviouslySuccessful
        );
        shooterDataPoints.put(currentDistance.in(Inches), dataPoint);
        successfulShotInput.set(false);
        shotsTakenAtCurrentVelocity++;

        System.out.println(
            "Shot taken at distance: " + currentDistance.in(Inches) + " inches = " + dataPoint.toString()
        );

        hasHitTargetAtCurrentDistance = hasHitTargetAtCurrentDistance || wasPreviouslySuccessful;

        // Read the distance override from the dashboard, if it is set
        if (distanceOverride.get() > 0) {
            currentDistance.mut_replace(distanceOverride.get(), Inches);
        }

        // Read velocity override from the dashboard, if it is set
        if (
            !currentAngularVelocityTarget.isNear(RadiansPerSecond.of(speedOverride.get()), ANGULAR_VELOCITY_TOLERANCE)
        ) {
            currentAngularVelocityTarget.mut_replace(speedOverride.get(), RadiansPerSecond);

            if (shotsTakenAtCurrentVelocity > 0) {
                System.out.println(
                    "Velocity override detected. Resetting shots taken at current velocity. New velocity: " +
                    currentAngularVelocityTarget.in(RPM) +
                    " RPM."
                );
                shotsTakenAtCurrentVelocity = 0;
            }
        }
        // If we have taken the required number of shots at the current velocity, increase the velocity
        else if (shotsTakenAtCurrentVelocity >= NUMBER_OF_SHOTS_PER_VELOCITY) {
            currentAngularVelocityTarget.mut_plus(VELOCITY_STEP);
            speedOverride.set(currentAngularVelocityTarget.in(RadiansPerSecond));
            shotsTakenAtCurrentVelocity = 0;
        }

        // If we have exceeded the ending velocity, reset to the starting velocity and increase the distance
        if (currentAngularVelocityTarget.gte(ENDING_ANGULAR_VELOCITY)) {
            incrementDistanceAndResetVelocity();
            return;
        }

        // If we have missed the target at the current distance and velocity, and we have already hit the target at this distance, go to next distance
        if (hasHitTargetAtCurrentDistance && !wasPreviouslySuccessful) {
            incrementDistanceAndResetVelocity();
            return;
        }
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetExitVelocity(currentAngularVelocityTarget);

        logState();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setTargetExitVelocity(RadiansPerSecond.zero());
        logState();
    }
}
