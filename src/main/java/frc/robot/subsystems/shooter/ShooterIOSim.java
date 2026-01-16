package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.swerve.Swerve;
import frc.util.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {

    private final Swerve swerveSubsystem;
    private final AbstractDriveTrainSimulation driveTrain;

    private LinearVelocity storedExitVelocity = MetersPerSecond.of(0.0);

    public ShooterIOSim(Swerve swerveSubsystem, AbstractDriveTrainSimulation driveTrain) {
        this.swerveSubsystem = swerveSubsystem;
        this.driveTrain = driveTrain;
    }

    @Override
    public void testShoot() {
        // note = fuel at home
        NoteOnFly fuelOnFly = new NoteOnFly(
            // Specify the position of the chassis when the note is launched
            swerveSubsystem.getActualPose().getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
            ShooterConstants.positionFromRobotCenter.getTranslation().toTranslation2d(),
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
            // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
            // velocityRPM / 6000 * 20,
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

        // fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
    }

    @Override
    public void setTargetExitVelocity(double velocityMetersPerSecond) {
        storedExitVelocity = MetersPerSecond.of(velocityMetersPerSecond);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {}
}
