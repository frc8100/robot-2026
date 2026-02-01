package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;

/**
 * The specification for the swerve drive. Implemented via a real swerve drive or a simulated swerve drive.
 */
public interface SwerveDrive extends Subsystem {
    /**
     * Drives the swerve modules based on the desired translation and rotation.
     * Should convert the translation and rotation to ChassisSpeeds and set the swerve modules to those speeds
     * in {@link #runVelocityChassisSpeeds}.
     * @param xMeters The desired x translation speed in meters per second.
     * @param yMeters The desired y translation speed in meters per second.
     * @param rotation The desired rotation speed.
     * @param fieldRelative Whether the speeds are field-relative.
     */
    public void drive(double xMeters, double yMeters, double rotation, boolean fieldRelative);

    /**
     * Drives the swerve modules given a provided robot-relative chassis speeds.
     * @param speed The desired chassis speeds
     */
    public void runVelocityChassisSpeeds(ChassisSpeeds speed);

    /**
     * Sets the desired states for the swerve modules. Used by SwerveControllerCommand in Auto.
     * @param desiredStates The desired states for the swerve modules.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates);

    /**
     * @return The current pose of the robot. This is determined by the swerve odometry.
     */
    public Pose2d getPose();

    /**
     * @return The current odometry rotation from {@link #getPose}
     */
    public default Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose);

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    );

    /**
     * @return The current module states.
     */
    public SwerveModuleState[] getModuleStates();

    /**
     * @return The current module positions.
     */
    public SwerveModulePosition[] getModulePositions();

    /**
     * @return The measured chassis speeds of the robot.
     */
    public ChassisSpeeds getChassisSpeeds();

    /**
     * Zeros the gyro.
     * @param deg The angle to zero the gyro to. Raw, without invert.
     */
    public void zeroGyro(double deg);

    public default void zeroGyro() {
        zeroGyro(0.0);
    }

    /**
     * @return The rotation of the robot for field-oriented control.
     * This can be different from the rotation of the robot from the pose estimator
     * and can be zeroed to any angle.
     */
    public Rotation2d getHeadingForFieldOriented();

    /** Stops the drive. */
    public default void stop() {
        runVelocityChassisSpeeds(new ChassisSpeeds());
    }
}
