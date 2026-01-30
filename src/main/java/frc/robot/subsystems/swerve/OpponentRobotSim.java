package frc.robot.subsystems.swerve;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Simulates another robot.
 * This simulation is not as detailed/complex as the main robot simulation.
 */
public class OpponentRobotSim extends SubsystemBase implements SwerveDrive {

    /**
     * Determines the behavior of opponent robots.
     * ! Note: this does not automatically set any commands.
     */
    public enum OpponentRobotBehavior {
        /**
         * Follows a path.
         * The default behavior.
         */
        FOLLOW_PATH,

        /**
         * Use teleop swerve.
         * More resource-intensive
         */
        TELEOP_SWERVE,

        /**
         * Continuously follow the main robot.
         */
        FOLLOW_MAIN_ROBOT,
    }

    /**
     * List of opponent robot poses.
     */
    private static ArrayList<Supplier<Pose2d>> opponentRobotPoses = new ArrayList<>();

    /**
     * @return The list of opponent robot poses for logging
     */
    @AutoLogOutput(key = "Odometry/OpponentRobotPoses")
    public static Pose2d[] getOpponentRobotPoses() {
        return opponentRobotPoses.stream().map(Supplier::get).toArray(Pose2d[]::new);
    }

    public final OpponentRobotBehavior behavior;

    private final RobotConfig pathPlannerConfig = SwerveConstants.getRobotConfig();
    private final PathConstraints pathPlannerPathConstraints = SwerveConstants.pathConstraints;
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.02),
        new PIDConstants(7.0, 0.05)
    );

    private final SelfControlledSwerveDriveSimulation simulatedDrive;

    public OpponentRobotSim(Pose2d startingPose, OpponentRobotBehavior behavior) {
        this.behavior = behavior;

        // Create the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(SwerveConstants.mapleSimConfig, startingPose)
        );

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // Add the pose supplier to the list
        opponentRobotPoses.add(this::getActualPose);
    }

    /**
     * @return A command to follow a path.
     */
    public Command opponentRobotFollowPath(PathPlannerPath path) {
        return new FollowPathCommand(
            path,
            // Provide actual robot pose in simulation, bypassing odometry error
            simulatedDrive::getActualPoseInSimulationWorld,
            // Provide actual robot speed in simulation, bypassing encoder measurement error
            simulatedDrive::getActualSpeedsRobotRelative,
            // Chassis speeds output
            (speeds, feedforwards) -> simulatedDrive.runChassisSpeeds(speeds, Translation2d.kZero, false, false),
            driveController,
            pathPlannerConfig,
            // Flip path based on alliance side
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red),
            this
        );
    }

    /**
     * @return A command to pathfind to a pose.
     */
    public Command opponentRobotPathfindToPose(Pose2d targetPose) {
        return new PathfindingCommand(
            targetPose,
            pathPlannerPathConstraints,
            simulatedDrive::getActualPoseInSimulationWorld,
            simulatedDrive::getActualSpeedsRobotRelative,
            (speeds, feedforwards) -> simulatedDrive.runChassisSpeeds(speeds, Translation2d.kZero, false, false),
            driveController,
            pathPlannerConfig,
            this
        );
    }

    /**
     * @return A command to recursively pathfind to a pose.
     */
    public Command opponentRobotPathfindToPoseSupplier(Supplier<Pose2d> poseSupplier) {
        return new PathfindingCommand(
            poseSupplier.get(),
            pathPlannerPathConstraints,
            simulatedDrive::getActualPoseInSimulationWorld,
            simulatedDrive::getActualSpeedsRobotRelative,
            (speeds, feedforwards) -> simulatedDrive.runChassisSpeeds(speeds, Translation2d.kZero, false, false),
            driveController,
            pathPlannerConfig,
            this
        )
            // Create a new command to recursively pathfind to the next pose every 1 second
            .withTimeout(1)
            .finallyDo(() -> {
                Command nextCommand = opponentRobotPathfindToPoseSupplier(poseSupplier);
                setDefaultCommand(nextCommand);
            });
    }

    @Override
    public void drive(double xMeters, double yMeters, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(xMeters, yMeters, rotation);

        this.simulatedDrive.runChassisSpeeds(speeds, Translation2d.kZero, true, true);
    }

    @Override
    public void runVelocityChassisSpeeds(ChassisSpeeds speed) {
        this.simulatedDrive.runChassisSpeeds(speed, Translation2d.kZero, true, true);
    }

    @Override
    public void setModuleStates(
        SwerveModuleState[] desiredStates,
        double[] feedforwardLinearForcesNewtons,
        double[] angleMotorVelocitiesRadPerSec
    ) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return simulatedDrive.getMeasuredStates();
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return simulatedDrive.getLatestModulePositions();
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        switch (behavior) {
            default:
            case FOLLOW_PATH:
                // Return actual pose to save resources
                return simulatedDrive.getActualSpeedsFieldRelative();
            case TELEOP_SWERVE:
                // Return accurate odometry pose
                return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
        }
    }

    @Override
    public Rotation2d getHeadingForFieldOriented() {
        // return simulatedDrive.getRawGyroAngle();
        // Returns the gyro angle in the simulation world to save resources
        return getPose().getRotation();
    }

    @Override
    public Pose2d getPose() {
        switch (behavior) {
            default:
            case FOLLOW_PATH:
                // Return actual pose to save resources
                return simulatedDrive.getActualPoseInSimulationWorld();
            case TELEOP_SWERVE:
                // Return accurate odometry pose
                return simulatedDrive.getOdometryEstimatedPose();
        }
    }

    @Override
    public Pose2d getActualPose() {
        return simulatedDrive.getActualPoseInSimulationWorld();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void zeroGyro(double deg) {
        // Set the pose but with a new rotation value
        setPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(deg)));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // Only update if teleop swerve is on
        if (behavior == OpponentRobotBehavior.TELEOP_SWERVE) {
            simulatedDrive.periodic();
        }
    }
}
