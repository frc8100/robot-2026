package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.swerve.Swerve;
import java.lang.reflect.Field;
import java.util.Optional;
import java.util.OptionalDouble;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;

public class ShooterIOSim extends ShooterIOYAMS {

    // Subsystem references
    private final Swerve swerveSubsystem;
    private final AbstractDriveTrainSimulation driveTrain;

    private LinearVelocity storedExitVelocity = MetersPerSecond.of(0.0);

    private static Field launchTimeField = null;

    /**
     * @return The Field object for the {@link GamePieceProjectile#calculatedHitTargetTime} field. Can be null if an error occurred.
     */
    private static Field getLaunchTimeField() {
        if (launchTimeField == null) {
            try {
                // Use reflection to access private field
                launchTimeField = GamePieceProjectile.class.getDeclaredField("calculatedHitTargetTime");
                launchTimeField.setAccessible(true); // NOSONAR
            } catch (NoSuchFieldException e) {
                e.printStackTrace();
            }
        }

        return launchTimeField;
    }

    public ShooterIOSim(Swerve swerveSubsystem, AbstractDriveTrainSimulation driveTrain) {
        super();
        this.swerveSubsystem = swerveSubsystem;
        this.driveTrain = driveTrain;
    }

    /**
     * Gets the launch time of the given projectile using reflection.
     * @param projectile - The projectile to get the launch time from.
     * @return The launch time in seconds, or empty if an error occurred.
     */
    private OptionalDouble getLaunchTime(GamePieceProjectile projectile) {
        try {
            Field field = getLaunchTimeField();
            if (field == null) {
                return OptionalDouble.empty();
            }

            double launchTime = (double) field.get(projectile);
            return OptionalDouble.of(launchTime);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
            return OptionalDouble.empty();
        }
    }

    @Override
    public void testShoot() {
        RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
            // Specify the position of the chassis when the note is launched
            swerveSubsystem.getActualPose().getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
            ShooterConstants.positionFromRobotCenter2d,
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
            driveTrain.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // The shooter facing direction is the same as the robot’s facing direction
            // ShooterConstants.positionFromRobotCenter
            //     .getRotation()
            //     .toRotation2d()
            //     .plus(swerveSubsystem.getActualPose().getRotation()),
            // TODO: this is temporarily set to ideal target
            new Rotation2d(swerveSubsystem.autoAim.latestCalculationResult.getRotationTarget()),
            // Initial height of the flying note
            ShooterConstants.positionFromRobotCenter.getMeasureZ(),
            // The launch speed
            storedExitVelocity,
            // The angle at which the note is launched
            ShooterConstants.exitAngle
        );

        fuelOnFly
            // Configure callbacks to visualize the flight trajectory of the projectile
            .withProjectileTrajectoryDisplayCallBack(
            // Callback for when the note will eventually hit the target (if configured)
            pose3ds -> Logger.recordOutput("Shooter/FuelTrajectoryHit", pose3ds.toArray(Pose3d[]::new)),
            pose3ds -> Logger.recordOutput("Shooter/FuelTrajectoryMiss", pose3ds.toArray(Pose3d[]::new))
        );

        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

        // Log the launch time
        OptionalDouble launchTimeOpt = getLaunchTime(fuelOnFly);
        if (launchTimeOpt.isPresent()) {
            Logger.recordOutput("Shooter/FuelLaunchTime", launchTimeOpt.getAsDouble());
        }
    }

    @Override
    public void setTargetExitVelocity(double velocityMetersPerSecond) {
        storedExitVelocity = MetersPerSecond.of(velocityMetersPerSecond);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        super.updateInputs(inputs);
    }
}
