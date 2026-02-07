package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.swerve.Swerve;
import frc.util.FuelSim;
import frc.util.FuelSim.Fuel;
import java.lang.reflect.Field;
import java.util.Optional;
import java.util.OptionalDouble;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import org.ironmaple.utils.FieldMirroringUtils;
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

    @Override
    public void testShoot() {
        Fuel fuel = FuelSim.getInstance()
            .launchFuel(
                storedExitVelocity,
                ShooterConstants.exitAngle.getMeasure(),
                swerveSubsystem.getActualPose().getRotation().plus(ShooterConstants.AIM_ROTATION_OFFSET).getMeasure(),
                // new Rotation2d(swerveSubsystem.autoAim.latestCalculationResult.getRotationTarget()),
                ShooterConstants.transformFromRobotCenter
            );

        // debug
        System.out.println("Launched fuel with exit velocity: " + storedExitVelocity);
    }

    @Override
    public void setTargetExitVelocity(double velocityMetersPerSecond) {
        storedExitVelocity = MetersPerSecond.of(velocityMetersPerSecond);
        // debug
        // System.out.println("Set target exit velocity to: " + storedExitVelocity);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        super.updateInputs(inputs);
    }
}
