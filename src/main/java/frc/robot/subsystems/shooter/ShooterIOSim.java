package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.swerve.Swerve;
import frc.util.FuelSim;
import frc.util.VelocityNoiseGenerator;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class ShooterIOSim extends ShooterIOYAMS {

    // Shoot timers
    private final Debouncer leftShootTimer = new Debouncer(
        1.0 / ShooterConstants.SIMULATION_MAX_FUEL_PER_SECOND,
        DebounceType.kRising
    );
    private final Debouncer rightShootTimer = new Debouncer(
        1.0 / ShooterConstants.SIMULATION_MAX_FUEL_PER_SECOND,
        DebounceType.kRising
    );

    private double leftTimeNoise = 0.0;
    private double rightTimeNoise = 0.0;

    /**
     * Filter to more closely model indexer acceleration.
     */
    private final LinearFilter indexerVelocityFilter = LinearFilter.movingAverage(5);

    private final VelocityNoiseGenerator shootTimeNoise = new VelocityNoiseGenerator(0.1, 0);

    // Subsystem references
    private final Swerve swerveSubsystem;
    private final Runnable onShoot;
    private final BooleanSupplier isAbleToShoot;

    public ShooterIOSim(Swerve swerveSubsystem, Runnable onShoot, BooleanSupplier isAbleToShoot) {
        super();
        this.swerveSubsystem = swerveSubsystem;
        this.onShoot = onShoot;
        this.isAbleToShoot = isAbleToShoot;

        super.leftShootMotorWrapped.setupCustomSimulation();
        super.indexerMotorWrapped.setupCustomSimulation();

        super.leftShootMotorWrapped.stopClosedLoopController();

        SmartMotorControllerConfig config = super.leftShootMotorWrapped.getConfig();
    }

    /**
     * Calculates the time until the next shot based on the indexer velocity. If the indexer is not moving, returns infinity. Otherwise, linearly interpolates the time until the next shot between 0 and the max fuel per second based on the indexer velocity (assuming max output of the indexer corresponds to max fuel per second).
     * @param indexerVelocity - The velocity of the indexer in radians per second.
     * @return The time until the next shot in seconds.
     */
    private double getWaitUntilNextShot(AngularVelocity indexerVelocity) {
        if (indexerVelocity.lt(RadiansPerSecond.of(15))) {
            // Indexer not moving, fuel never shoots
            return Double.POSITIVE_INFINITY;
        } else {
            // Linearly interpolate the time until the next shot between 0 and the max fuel per second based on the indexer velocity (assuming max output of the indexer corresponds to max fuel per second)
            double filteredIndexerVelocity = indexerVelocityFilter.calculate(indexerVelocity.in(RadiansPerSecond));

            return (
                (1.0 /
                    (MathUtil.clamp(
                            MathUtil.inverseInterpolate(
                                0.0,
                                ShooterConstants.SIMULATION_INDEXER_VELOCITY_AT_MAX_OUTPUT.in(RadiansPerSecond),
                                filteredIndexerVelocity
                            ),
                            0.0,
                            1.0
                        ) *
                        ShooterConstants.SIMULATION_MAX_FUEL_PER_SECOND))
            );
        }
    }

    /**
     * Launches a fuel with the given exit velocity and the shooter's configured exit angle.
     * @param exitVelocity - The velocity at which the fuel should exit the shooter in meters per second.
     */
    private void shootFuelWithVelocity(LinearVelocity exitVelocity, Transform3d fuelStreamTransform) {
        FuelSim.getInstance()
            .launchFuel(
                exitVelocity,
                ShooterConstants.exitAngle.getMeasure(),
                swerveSubsystem.getActualPose().getRotation().plus(ShooterConstants.AIM_ROTATION_OFFSET).getMeasure(),
                // new Rotation2d(swerveSubsystem.autoAim.latestCalculationResult.getRotationTarget()),
                ShooterConstants.transformFromRobotCenter.plus(fuelStreamTransform)
            );

        onShoot.run();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        super.updateInputs(inputs);

        // Shoot if time has passed based on the indexer velocity
        double leftTimeUntilNextShot = getWaitUntilNextShot(inputs.indexerMotorData.velocity);
        Logger.recordOutput("Shooter/TimeUntilNextShot", leftTimeUntilNextShot);

        // If the time until the next shot is infinity, reset the shoot timer so that it will not shoot immediately once the indexer starts moving
        if (leftTimeUntilNextShot == Double.POSITIVE_INFINITY) {
            leftShootTimer.calculate(false);
        }
        leftShootTimer.setDebounceTime(leftTimeUntilNextShot + leftTimeNoise);

        if (isAbleToShoot.getAsBoolean() && leftShootTimer.calculate(true)) {
            // Reset timer
            leftShootTimer.calculate(false);
            leftTimeNoise = shootTimeNoise.generateNoise(0.0);

            shootFuelWithVelocity(
                MetersPerSecond.of(
                    Shooter.getCurrentPredictedFuelExitVelocityFromMotor(inputs.shootMotorData.velocity)
                ),
                ShooterConstants.leftFuelStreamTransform
            );
        }

        double rightTimeUntilNextShot = getWaitUntilNextShot(inputs.indexerMotorData.velocity);
        rightShootTimer.setDebounceTime(rightTimeUntilNextShot + rightTimeNoise);

        if (isAbleToShoot.getAsBoolean() && rightShootTimer.calculate(true)) {
            // Reset timer
            rightShootTimer.calculate(false);
            rightTimeNoise = shootTimeNoise.generateNoise(0.0);

            shootFuelWithVelocity(
                MetersPerSecond.of(
                    Shooter.getCurrentPredictedFuelExitVelocityFromMotor(inputs.shootMotorData.velocity)
                ),
                ShooterConstants.rightFuelStreamTransform
            );
        }
    }

    @Override
    public void simIterate() {
        super.flywheel.simIterate();
        super.indexerMotorWrapped.simIterate();
    }
}
