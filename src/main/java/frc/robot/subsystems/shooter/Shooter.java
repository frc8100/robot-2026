package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.AimToTarget;
import frc.robot.subsystems.swerve.Swerve;
import frc.util.FuelSim;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem.
 */
public class Shooter extends SubsystemBase {

    /**
     * Gets the current predicted fuel exit velocity based on the current motor angular velocity using the distance lookup table as an intermediary.
     * @param motorAngularVelocity - The current motor angular velocity in radians per second.
     * @return The predicted fuel exit velocity in meters per second.
     */
    public static double getCurrentPredictedFuelExitVelocityFromMotor(AngularVelocity motorAngularVelocity) {
        double predictedDistanceMeters = motorAngularVelocityToDistanceMap.get(
            motorAngularVelocity.in(RadiansPerSecond)
        );

        return AimToTarget.distanceToExitVelocityMap.get(predictedDistanceMeters);
    }

    // Inverse maps
    private static final InterpolatingDoubleTreeMap motorAngularVelocityToDistanceMap =
        ShooterConstants.distanceToMotorAngularVelocityMap.getInverseMap();

    public enum ShooterState {
        /**
         * Not shooting.
         */
        IDLE,

        /**
         * The shooter is spinning to the target velocity and the indexer is running to move fuel to the shooter.
         */
        SHOOTING,
    }

    public final StateMachine<ShooterState, Object> stateMachine = new StateMachine<ShooterState, Object>(
        ShooterState.class,
        "Shooter"
    )
        .withDefaultState(new StateMachineState<>(ShooterState.IDLE, "Idle"))
        .withState(new StateMachineState<>(ShooterState.SHOOTING, "Shooting"))
        .withReturnToDefaultStateOnDisable(true);

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Subsystem references
    private final Swerve swerveSubsystem;

    // Caches
    private final MutAngularVelocity cachedTargetExitAngularVelocity = RadiansPerSecond.mutable(0.0);

    private final SysIdRoutine shooterSysidRoutine;

    /**
     * Stores the result of a fuel trajectory prediction, including the trajectory points, the final point, and whether it would hit the target.
     */
    public static class CachedPredictedTrajectoryResult {

        public final List<Translation3d> trajectoryPoints = new ArrayList<>(30);
        public Translation3d finalPoint = Translation3d.kZero;
        public boolean hitTarget = false;

        protected CachedPredictedTrajectoryResult() {}

        protected void log(String prefix) {
            Logger.recordOutput(prefix + "/Trajectory", trajectoryPoints.toArray(new Translation3d[0]));
            Logger.recordOutput(prefix + "/FinalPoint", finalPoint);
            Logger.recordOutput(prefix + "/HitTarget", hitTarget);
        }
    }

    public final CachedPredictedTrajectoryResult cachedTargetTrajectoryResult = new CachedPredictedTrajectoryResult();
    public final CachedPredictedTrajectoryResult cachedCurrentTrajectoryResult = new CachedPredictedTrajectoryResult();
    public final CachedPredictedTrajectoryResult cachedFutureTrajectoryResult = new CachedPredictedTrajectoryResult();

    public Shooter(ShooterIO io, Swerve swerveSubsystem) {
        this.io = io;
        this.swerveSubsystem = swerveSubsystem;

        // State machine bindings
        stateMachine.whileState(ShooterState.IDLE, this::handleIdleState);
        stateMachine.whileState(ShooterState.SHOOTING, this::handleShootState);

        setDefaultCommand(stateMachine.getRunnableCommand(this));

        shooterSysidRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                ShooterConstants.SHOOTER_SYSID_RAMP_RATE,
                ShooterConstants.SHOOTER_SYSID_MAX_VOLTAGE,
                ShooterConstants.SHOOTER_SYSID_TEST_DURATION,
                state -> Logger.recordOutput("Shooter/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    io.runShooterDutyCycle(voltage);
                    io.runIndexerDutyCycle(voltage);
                },
                null,
                this
            )
        );
    }

    public Command runShooterDutyCycle(Voltage dutyCycleOutput) {
        return run(() -> io.runShooterDutyCycle(dutyCycleOutput));
    }

    /**
     * Handles the logic for the {@link ShooterState#IDLE} state which stops the shooter and indexer.
     */
    private void handleIdleState() {
        io.stopShooter();
        io.stopIndexer();
    }

    /**
     * Handles the logic for the {@link ShooterState#SHOOTING} state which sets the target exit velocity based on the distance to the target.
     */
    private void handleShootState() {
        setTargetExitVelocity(swerveSubsystem.autoAim.latestCalculationResult.getDistanceToTarget());

        boolean shooterUpToSpeed = cachedCurrentTrajectoryResult.hitTarget;

        // TODO: add override
        boolean isBeingOverrided = false;

        if (shooterUpToSpeed || isBeingOverrided) {
            io.setIndexerVelocitySetpoint(ShooterConstants.INDEXER_SPEED);
        }
    }

    /**
     * Predicts the trajectory of a fuel based on the current shooter exit velocity, angle, and robot velocity. Also predicts whether the fuel would score in the hub.
     */
    private void updatePredictedFuelTrajectory(
        CachedPredictedTrajectoryResult result,
        double exitVelocityMPS,
        boolean shouldLogExtraData
    ) {
        result.trajectoryPoints.clear();

        // Precompute constants
        final Rotation2d shooterAngle = swerveSubsystem
            .getPose()
            .getRotation()
            .plus(ShooterConstants.AIM_ROTATION_OFFSET);
        final ChassisSpeeds robotVelocity = swerveSubsystem.getFieldRelativeSpeeds();

        // Calculate the initial velocity vector based on the exit velocity and angle
        Translation3d velocity = new Translation3d(
            exitVelocityMPS * ShooterConstants.exitAngle.getCos() * shooterAngle.getCos() +
            robotVelocity.vxMetersPerSecond,
            exitVelocityMPS * ShooterConstants.exitAngle.getCos() * shooterAngle.getSin() +
            robotVelocity.vyMetersPerSecond,
            exitVelocityMPS * ShooterConstants.exitAngle.getSin()
        );

        Translation3d position = new Pose3d(swerveSubsystem.getPose())
            .transformBy(ShooterConstants.transformFromRobotCenter)
            .getTranslation();

        // Predict final position at time to target
        double finalT =
            swerveSubsystem.autoAim.latestCalculationResult.getTimeToTarget().in(Seconds) +
            ShooterConstants.PREDICT_FUEL_POSITION_LOOKAHEAD_TIME.in(Seconds);
        Translation3d finalPosition = new Translation3d(
            position.getX() + velocity.getX() * finalT,
            position.getY() + velocity.getY() * finalT,
            position.getZ() +
            velocity.getZ() * finalT -
            0.5 * ShooterConstants.g.in(MetersPerSecondPerSecond) * finalT * finalT
        );
        Translation3d velocityAtFinalPosition = new Translation3d(
            velocity.getX(),
            velocity.getY(),
            velocity.getZ() - ShooterConstants.g.in(MetersPerSecondPerSecond) * finalT
        );

        // Check if the predicted final position would score in the hub
        FuelSim.Fuel predictedFuel = new FuelSim.Fuel(finalPosition, velocityAtFinalPosition);
        boolean hitTarget =
            // Check both hubs because we aren't aiming at the opponent's hub (hopefully)
            FuelSim.Hub.BLUE_HUB.didFuelScoreAtAll(predictedFuel) ||
            FuelSim.Hub.RED_HUB.didFuelScoreAtAll(predictedFuel);

        result.finalPoint = finalPosition;
        result.hitTarget = hitTarget;

        // If we don't need to log additional data, skip calculating the full trajectory for performance reasons
        if (!shouldLogExtraData) {
            result.finalPoint = finalPosition;
            result.hitTarget = hitTarget;
        }

        for (int i = 0; i < 30; i++) {
            double t = i * 0.1;

            // If we've reached the final time, add the final position and break
            if (t > finalT) {
                result.trajectoryPoints.add(finalPosition);

                break;
            }

            // Calculate with simple projectile motion equations with gravity
            double z =
                position.getZ() + velocity.getZ() * t - 0.5 * ShooterConstants.g.in(MetersPerSecondPerSecond) * t * t;

            // If below ground level, stop the trajectory
            if (z < 0) {
                result.trajectoryPoints.add(
                    new Translation3d(position.getX() + velocity.getX() * t, position.getY() + velocity.getY() * t, 0)
                );
                break;
            }

            double x = position.getX() + velocity.getX() * t;
            double y = position.getY() + velocity.getY() * t;

            // Update position for the next iteration
            result.trajectoryPoints.add(new Translation3d(x, y, z));
        }
    }

    /**
     * Sets the target exit velocity for the shooter. Does not look up.
     * @param targetExitVelocity - The target exit velocity in radians per second of the shooter motor angular velocity.
     */
    public void setTargetExitVelocity(AngularVelocity targetExitVelocity) {
        io.setTargetShootMotorVelocity(targetExitVelocity);
    }

    /**
     * Sets the target exit velocity for the shooter based on the distance to the target.
     * @param distanceToTarget - The distance to the target in meters.
     */
    public void setTargetExitVelocity(Distance distanceToTarget) {
        // Look up the corresponding motor angular velocity for the given distance and update cache
        cachedTargetExitAngularVelocity.mut_replace(
            ShooterConstants.distanceToMotorAngularVelocityMap.get(distanceToTarget.in(Meters)),
            RadiansPerSecond
        );

        io.setTargetShootMotorVelocity(cachedTargetExitAngularVelocity);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        // Log trajectory points for visualization
        updatePredictedFuelTrajectory(
            cachedTargetTrajectoryResult,
            swerveSubsystem.autoAim.latestCalculationResult.getTargetFuelExitVelocity().in(MetersPerSecond),
            Constants.shouldLogAdditionalData()
        );
        cachedTargetTrajectoryResult.log("Shooter/TargetTrajectory");

        updatePredictedFuelTrajectory(
            cachedCurrentTrajectoryResult,
            getCurrentPredictedFuelExitVelocityFromMotor(inputs.shootMotorData.velocity),
            Constants.shouldLogAdditionalData()
        );
        cachedCurrentTrajectoryResult.log("Shooter/CurrentTrajectory");

        updatePredictedFuelTrajectory(
            cachedFutureTrajectoryResult,
            getCurrentPredictedFuelExitVelocityFromMotor(inputs.shootMotorData.velocity) + 0.2,
            Constants.shouldLogAdditionalData()
        );
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }

    /**
     * @return A command that runs all 4 sysid routines.
     */
    public Command shooterSysidCommand() {
        return new SequentialCommandGroup(
            shooterSysidRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(4),
            shooterSysidRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(4),
            shooterSysidRoutine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(4),
            shooterSysidRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    // Getters
    public AngularVelocity getCurrentMotorAngularVelocity() {
        return inputs.shootMotorData.velocity;
    }
}
