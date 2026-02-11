package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
        .withState(new StateMachineState<>(ShooterState.SHOOTING, "Shooting"));

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Subsystem references
    private final Swerve swerveSubsystem;

    // Caches
    private final List<Translation3d> trajectoryPoints = new ArrayList<>(30);
    private final MutAngularVelocity cachedTargetExitAngularVelocity = RadiansPerSecond.mutable(0.0);

    private final SysIdRoutine shooterSysidRoutine;

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
            new SysIdRoutine.Mechanism(io::runShooterDutyCycle, null, this)
        );
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

        // TODO: add check to only run indexer if shooter is up to speed
        io.runIndexer();
    }

    /**
     * Stores the result of a fuel trajectory prediction, including the trajectory points, the final point, and whether it would hit the target.
     */
    public record PredictedTrajectoryResult(
        Translation3d[] trajectoryPoints,
        Translation3d finalPoint,
        boolean hitTarget
    ) {
        public void log(String prefix) {
            Logger.recordOutput(prefix + "/Trajectory", trajectoryPoints);
            Logger.recordOutput(prefix + "/FinalPoint", finalPoint);
            Logger.recordOutput(prefix + "/HitTarget", hitTarget);
        }
    }

    /**
     * Predicts the trajectory of a fuel based on the current shooter exit velocity, angle, and robot velocity. Also predicts whether the fuel would score in the hub.
     */
    public PredictedTrajectoryResult predictFuelTrajectory(double exitVelocityMPS) {
        trajectoryPoints.clear();

        // Precompute constants
        final double exitAngleRad = ShooterConstants.exitAngle.getRadians();
        final double shooterAngleRad = swerveSubsystem
            .getPose()
            .getRotation()
            .plus(ShooterConstants.AIM_ROTATION_OFFSET)
            .getRadians();
        final ChassisSpeeds robotVelocity = swerveSubsystem.getFieldRelativeSpeeds();

        // Calculate the initial velocity vector based on the exit velocity and angle
        Translation3d velocity = new Translation3d(
            exitVelocityMPS * Math.cos(exitAngleRad) * Math.cos(shooterAngleRad) + robotVelocity.vxMetersPerSecond,
            exitVelocityMPS * Math.cos(exitAngleRad) * Math.sin(shooterAngleRad) + robotVelocity.vyMetersPerSecond,
            exitVelocityMPS * Math.sin(exitAngleRad)
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

        // If we don't need to log additional data, skip calculating the full trajectory for performance reasons
        if (!Constants.shouldLogAdditionalData()) {
            return new PredictedTrajectoryResult(new Translation3d[0], finalPosition, hitTarget);
        }

        for (int i = 0; i < 30; i++) {
            double t = i * 0.1;

            // If we've reached the final time, add the final position and break
            if (t > finalT) {
                trajectoryPoints.add(finalPosition);
                break;
            }

            // Calculate with simple projectile motion equations with gravity
            double z =
                position.getZ() + velocity.getZ() * t - 0.5 * ShooterConstants.g.in(MetersPerSecondPerSecond) * t * t;

            // If below ground level, stop the trajectory
            if (z < 0) {
                trajectoryPoints.add(
                    new Translation3d(position.getX() + velocity.getX() * t, position.getY() + velocity.getY() * t, 0)
                );
                break;
            }

            double x = position.getX() + velocity.getX() * t;
            double y = position.getY() + velocity.getY() * t;

            // Update position for the next iteration
            trajectoryPoints.add(new Translation3d(x, y, z));
        }

        return new PredictedTrajectoryResult(trajectoryPoints.toArray(new Translation3d[0]), finalPosition, hitTarget);
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
        PredictedTrajectoryResult targetTrajectory = predictFuelTrajectory(
            swerveSubsystem.autoAim.latestCalculationResult.getTargetFuelExitVelocity().in(MetersPerSecond)
        );
        targetTrajectory.log("Shooter/TargetTrajectory");

        PredictedTrajectoryResult currentTrajectory = predictFuelTrajectory(
            getCurrentPredictedFuelExitVelocityFromMotor(inputs.shootMotorData.velocity)
        );
        currentTrajectory.log("Shooter/CurrentTrajectory");
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }

    public Command shooterSysidCommand() {
        return new SequentialCommandGroup(
            shooterSysidRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1),
            shooterSysidRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(1),
            shooterSysidRoutine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1),
            shooterSysidRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }
}
