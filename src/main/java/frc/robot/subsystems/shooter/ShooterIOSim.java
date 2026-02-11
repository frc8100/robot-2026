package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.util.FuelSim;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim extends ShooterIOYAMS {

    // Shoot timers
    private final Debouncer shootTimer = new Debouncer(
        1.0 / ShooterConstants.SIMULATION_MAX_FUEL_PER_SECOND,
        DebounceType.kRising
    );
    /**
     * Filter to more closely model indexer acceleration.
     */
    private final LinearFilter indexerVelocityFilter = LinearFilter.movingAverage(5);

    // Subsystem references
    private final Swerve swerveSubsystem;
    private final Runnable onShoot;
    private final BooleanSupplier isAbleToShoot;

    // TODO: Add PID controller for shooter motor velocity because YAMS does not simulate that
    private final ProfiledPIDController shootMotorVelocityController = super.shootMotorWrapped
        .getConfig()
        .getClosedLoopController()
        .orElseThrow();
    private final SimpleMotorFeedforward shootMotorFeedforward = super.shootMotorWrapped
        .getConfig()
        .getSimpleFeedforward()
        .orElseThrow();
    private boolean isClosedLoopControllerEnabled = false;
    private double previousVelocitySetpointRPS = 0.0;

    public ShooterIOSim(Swerve swerveSubsystem, Runnable onShoot, BooleanSupplier isAbleToShoot) {
        super();
        this.swerveSubsystem = swerveSubsystem;
        this.onShoot = onShoot;
        this.isAbleToShoot = isAbleToShoot;

        super.shootMotorWrapped.setupCustomSimulation();
        super.indexerMotorWrapped.setupCustomSimulation();

        super.shootMotorWrapped.stopClosedLoopController();
    }

    @Override
    public void setTargetShootMotorVelocity(AngularVelocity velocity) {
        super.shootMotorWrapped.setSetpointVelocity(velocity);
        isClosedLoopControllerEnabled = true;
    }

    @Override
    public void stopShooter() {
        super.stopShooter();
        isClosedLoopControllerEnabled = false;
        previousVelocitySetpointRPS = 0.0;
    }

    @Override
    public void runShooterDutyCycle(Voltage dutyCycleOutput) {
        super.runShooterDutyCycle(dutyCycleOutput);
        isClosedLoopControllerEnabled = false;
        previousVelocitySetpointRPS = 0.0;
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
                1.0 /
                (MathUtil.clamp(
                        MathUtil.inverseInterpolate(
                            0.0,
                            ShooterConstants.SIMULATION_INDEXER_VELOCITY_AT_MAX_OUTPUT.in(RadiansPerSecond),
                            filteredIndexerVelocity
                        ),
                        0.0,
                        1.0
                    ) *
                    ShooterConstants.SIMULATION_MAX_FUEL_PER_SECOND)
            );
        }
    }

    /**
     * Launches a fuel with the given exit velocity and the shooter's configured exit angle.
     * @param exitVelocity - The velocity at which the fuel should exit the shooter in meters per second.
     */
    private void shootFuelWithVelocity(LinearVelocity exitVelocity) {
        FuelSim.getInstance()
            .launchFuel(
                exitVelocity,
                ShooterConstants.exitAngle.getMeasure(),
                swerveSubsystem.getActualPose().getRotation().plus(ShooterConstants.AIM_ROTATION_OFFSET).getMeasure(),
                // new Rotation2d(swerveSubsystem.autoAim.latestCalculationResult.getRotationTarget()),
                ShooterConstants.transformFromRobotCenter
            );

        onShoot.run();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        super.updateInputs(inputs);

        // test
        Logger.recordOutput("Shooter/ShooterSupplyCurrent", super.shootMotorWrapped.getCustomSupplyCurrent());

        // Iterate the shoot motor velocity controller to get the motor output
        if (isClosedLoopControllerEnabled) {
            double currentShooterVelocityRPS = inputs.shootMotorData.velocity.in(RotationsPerSecond);
            double setpointShooterVelocityRPS = super.shootMotorWrapped
                .getMechanismSetpointVelocity()
                .orElse(RotationsPerSecond.zero())
                .in(RotationsPerSecond);

            double shootMotorOutput =
                shootMotorVelocityController.calculate(currentShooterVelocityRPS, setpointShooterVelocityRPS) +
                shootMotorFeedforward.calculateWithVelocities(previousVelocitySetpointRPS, setpointShooterVelocityRPS);
            super.shootMotorWrapped.setVoltage(Volts.of(shootMotorOutput));

            previousVelocitySetpointRPS = setpointShooterVelocityRPS;
        }

        // Shoot if time has passed based on the indexer velocity
        double timeUntilNextShot = getWaitUntilNextShot(inputs.indexerMotorData.velocity);
        Logger.recordOutput("Shooter/TimeUntilNextShot", timeUntilNextShot);

        // If the time until the next shot is infinity, reset the shoot timer so that it will not shoot immediately once the indexer starts moving
        if (timeUntilNextShot == Double.POSITIVE_INFINITY) {
            shootTimer.calculate(false);
        }
        shootTimer.setDebounceTime(timeUntilNextShot);

        if (isAbleToShoot.getAsBoolean() && shootTimer.calculate(true)) {
            // Reset timer
            shootTimer.calculate(false);

            shootFuelWithVelocity(
                MetersPerSecond.of(Shooter.getCurrentPredictedFuelExitVelocityFromMotor(inputs.shootMotorData.velocity))
            );
        }
    }

    @Override
    public void simIterate() {
        super.shootMotorWrapped.simIterate();
        super.indexerMotorWrapped.simIterate();
    }
}
