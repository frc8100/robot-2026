package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ControlConstants;
import frc.robot.commands.AimToTarget;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.util.LocalADStarAK;
import frc.util.SwerveFeedForwards;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import java.util.OptionalDouble;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve subsystem, responsible for controlling the swerve drive.
 */
public class Swerve extends SubsystemBase {

    /**
     * Configures the path planner auto builder and records the path and trajectory setpoint to the logger.
     */
    public static void configurePathPlannerAutoBuilder(Swerve swerveSubsystem, QuestNavSubsystem questNavSubsystem) {
        AutoBuilder.configure(
            swerveSubsystem::getPose,
            (Pose2d newPose) -> {
                swerveSubsystem.setPose(newPose);
                questNavSubsystem.setPose(newPose);
            },
            swerveSubsystem::getChassisSpeeds,
            swerveSubsystem::runVelocityChassisSpeeds,
            SwerveConstants.PP_INITIAL_PID_CONTROLLER,
            SwerveConstants.getRobotConfig(),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            swerveSubsystem
        );

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(activePath ->
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]))
        );
        PathPlannerLogging.setLogTargetPoseCallback(targetPose ->
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        );
    }

    /**
     * Lock for the odometry thread.
     */
    public static final Lock odometryLock = new ReentrantLock();

    private final TeleopSwerve teleopSwerve;

    /**
     * The auto-aim command.
     */
    public final AimToTarget autoAim = new AimToTarget();

    public enum SwerveState {
        // TODO: doc
        // IDLE,

        /**
         * The driver has full control over swerve. No autonomous actions are taken.
         */
        FULL_DRIVER_CONTROL,

        /**
         * The drive can control translation while the robot auto-aims to a target.
         */
        AUTO_AIM,

        /**
         * The robot is driving to a target pose.
         * The driver has partial control over swerve and can nudge the robot in a direction.
         */

        /**
         * The robot is performing initial pathfinding to the target pose.
         */
        DRIVE_TO_POSE_PATHFINDING,

        /**
         * The robot is performing final alignment to the target pose using a simple PID controller.
         */
        DRIVE_TO_POSE_PID,

        /**
         * The robot has reached the target pose.
         */
        DRIVE_TO_POSE_AT_TARGET,

        /**
         * The robot is fully autonomous and following a pre-planned path.
         */
        FULL_AUTONOMOUS_PATH_FOLLOWING,
    }

    /**
     * The payload for the swerve state machine.
     * @param poseSupplier - The supplier of the target pose.
     * @param shouldRotateToPoseSupplier - The supplier of whether the robot should rotate to the target pose. See {@link SwervePayload.RotationMode}.
     * @param poseToRotateToSupplier - The supplier of the pose to rotate to.
     */
    public record SwervePayload(
        Supplier<APTarget> poseSupplier,
        Supplier<SwervePayload.RotationMode> shouldRotateToPoseSupplier,
        Supplier<Pose2d> poseToRotateToSupplier
    ) {
        /**
         * The rotation mode for the swerve drive when driving to a pose.
         */
        public enum RotationMode {
            /**
             * Only rotate to the target pose, do not drive to the target pose.
             */
            ONLY_ROTATE_TO_POSE_NO_DRIVE_TO_POSE,

            /**
             * Rotate and drive to the target pose.
             */
            ROTATE_AND_DRIVE_TO_POSE,

            /**
             * Only drive to the target pose, do not rotate to the target pose.
             * Default mode.
             */
            ONLY_DRIVE_NO_ROTATE,
        }

        /**
         * Creates a SwervePayload that does not rotate to the target pose.
         * @param poseSupplier - The supplier of the target pose.
         */
        public static SwervePayload fromPoseSupplierNoRotate(Supplier<Pose2d> poseSupplier) {
            return new SwervePayload(
                () -> new APTarget(poseSupplier.get()),
                () -> RotationMode.ONLY_DRIVE_NO_ROTATE,
                () -> Pose2d.kZero
            );
        }

        public static SwervePayload fromAPTargetSupplierNoRotate(Supplier<APTarget> apTargetSupplier) {
            return new SwervePayload(apTargetSupplier, () -> RotationMode.ONLY_DRIVE_NO_ROTATE, () -> Pose2d.kZero);
        }
    }

    public boolean isSimulation() {
        return false;
    }

    /**
     * The state machine for the swerve subsystem.
     * The payload is the target pose for the robot when in {@link SwerveState#DRIVE_TO_POSE}.
     */
    public final StateMachine<SwerveState, SwervePayload> stateMachine = new StateMachine<SwerveState, SwervePayload>(
        SwerveState.class,
        "Swerve"
    )
        .withDefaultState(new StateMachineState<>(SwerveState.FULL_DRIVER_CONTROL, "Manual"))
        // .withState(new StateMachineState<>(SwerveState.IDLE, "Idle"))
        .withState(new StateMachineState<>(SwerveState.AUTO_AIM, "AutoAim"))
        .withState(new StateMachineState<>(SwerveState.DRIVE_TO_POSE_PATHFINDING, "InitialPathfinding"))
        .withState(new StateMachineState<>(SwerveState.DRIVE_TO_POSE_PID, "PIDAlignment"))
        .withState(new StateMachineState<>(SwerveState.DRIVE_TO_POSE_AT_TARGET, "AtTarget"))
        .withState(new StateMachineState<>(SwerveState.FULL_AUTONOMOUS_PATH_FOLLOWING, "FollowPath"))
        .withReturnToDefaultStateOnDisable(true);

    // TODO: In replay, this::isSimulation does not capture correctly; fix this
    private final SwerveFeedForwards swerveFeedForwards = new SwerveFeedForwards(this::isSimulation);

    /**
     * The setpoint generator for the swerve drive. See https://pathplanner.dev/pplib-swerve-setpoint-generator.html for more info.
     */
    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
        SwerveConstants.getRobotConfig(),
        SwerveConstants.MAX_ANGULAR_VELOCITY_OF_SWERVE_MODULE
    );

    /**
     * Previous setpoints used for {@link #setpointGenerator}.
     */
    // TODO: temporarily public for testing; make private later
    private SwerveSetpoint moduleStateSetpoint = null;

    private ChassisSpeeds setpointSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    /**
     * The swerve modules. These are the four swerve modules on the robot. Each module has a drive
     * motor and a steering motor.
     */
    private final Module[] swerveModules = new Module[4];

    /**
     * The gyro. This is used to determine the robot's heading.
     */
    public final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    /**
     * Raw gyro rotation. Used for the pose estimator.
     */
    private Rotation2d rawGyroRotation = new Rotation2d();

    /**
     * Kinematics for the swerve drive. Used to convert between chassis speeds and module states.
     */
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATIONS);

    /**
     * The last stored position of the swerve modules for delta tracking.
     */
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };

    public final SwerveDrivePoseEstimator poseEstimator;

    /**
     * A flag to indicate whether to run the chassis speeds this cycle.
     * When {@link #runVelocityChassisSpeeds} is called, this is set to true.
     * At the end of {@link #periodic}, this is reset to false.
     */
    private boolean shouldRunSpeedsThisCycle = true;

    /**
     * The yaw offset for field-oriented driving.
     */
    private Rotation2d yawOffset = Rotation2d.kZero;

    // SysId routines for drive and angle motors
    protected SysIdRoutine driveSysId = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, state -> Logger.recordOutput("Swerve/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(voltage -> runCharacterization(voltage.in(Volt)), null, this)
    );
    protected SysIdRoutine angleSysId = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, state ->
            Logger.recordOutput("Swerve/AngleSysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            voltage -> {
                for (int i = 0; i < 4; i++) {
                    swerveModules[i].runAngleCharacterization(voltage.in(Volt));
                }
            },
            null,
            this
        )
    );

    /**
     * A trigger that syncs the motor encoders to the absolute encoders when the robot is still for a certain time.
     */
    private final Trigger syncMotorEncodersToAbsoluteEncoderTrigger = new Trigger(
        () -> getVelocityMagnitudeAsDouble() < SwerveConstants.STILL_MPS
    ).debounce(SwerveConstants.TIME_AFTER_STILL_SYNC_ENCODERS.in(Seconds));

    /**
     * Creates a new Swerve subsystem.
     * @param gyroIO - The gyro IO implementation.
     * @param moduleIOs - The module IO implementations for each of the four swerve modules.
     */
    public Swerve(GyroIO gyroIO, ModuleIO[] moduleIOs) {
        // Create the swerve modules
        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new Module(moduleIOs[i], i);
        }

        this.gyroIO = gyroIO;

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            SwerveConstants.initialPose,
            SwerveConstants.stateStdDevs,
            SwerveConstants.visionStdDevs
        );

        // TODO: store this in constants
        zeroYawOffset(Rotation2d.k180deg);

        teleopSwerve = new TeleopSwerve(
            this,
            // Switch between joystick and main drive controls depending on the mode
            ControlConstants.USE_JOYSTICK_DRIVE
                ? ControlConstants.joystickDriveControls
                : ControlConstants.mainDriveControls,
            true
        );

        // TODO: Sync motor encoders to absolute encoders when the robot is still
        // syncMotorEncodersToAbsoluteEncoderTrigger.onTrue(
        //     Commands.runOnce(() -> {
        //         for (Module module : swerveModules) {
        //             module.syncMotorEncoderToAbsoluteEncoder();
        //         }

        //         // debug
        //         System.out.println("Swerve motor encoders synced");
        //     })
        // );

        // Start odometry thread
        OdometryThread.getInstance().start();
    }

    /**
     * @return The chassis speeds from a translation and rotation input, either field-relative or robot-relative.
     */
    public ChassisSpeeds getSpeedsFromTranslation(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadPerSec,
        boolean fieldRelative
    ) {
        // Determine the desired chassis speeds based on whether the control is field-relative
        return fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadPerSec,
                getHeadingForFieldOriented()
            )
            : new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadPerSec);
    }

    /**
     * Drives the swerve modules based on the desired translation and rotation.
     * Should convert the translation and rotation to ChassisSpeeds and set the swerve modules to those speeds
     * in {@link #runVelocityChassisSpeeds}.
     * @param xMeters - The desired x translation speed in meters per second.
     * @param yMeters - The desired y translation speed in meters per second.
     * @param rotation - The desired rotation speed.
     * @param fieldRelative - Whether the speeds are field-relative.
     */
    public void drive(double xMeters, double yMeters, double omegaRadPerSec, boolean fieldRelative) {
        ChassisSpeeds desiredChassisSpeeds = getSpeedsFromTranslation(xMeters, yMeters, omegaRadPerSec, fieldRelative);

        runVelocityChassisSpeeds(desiredChassisSpeeds);
    }

    /**
     * Drives the swerve modules given a provided robot-relative chassis speeds.
     * @param speed The desired chassis speeds
     */
    public void runVelocityChassisSpeeds(ChassisSpeeds speed) {
        setpointSpeeds = speed;
        shouldRunSpeedsThisCycle = true;
    }

    /**
     * Internal method to calculate and run the swerve module states based on the current setpoints by {@link #runVelocityChassisSpeeds}.
     */
    @SuppressWarnings("unused")
    private void runSpeeds() {
        // Skip if previous setpoint is null (should only happen on first run)
        if (moduleStateSetpoint == null) {
            return;
        }

        // Apply anti-tipping correction
        if (SwerveConstants.IS_ANTI_TIPPING_ENABLED && gyroInputs.isTipping) {
            ChassisSpeeds antiTippingSpeeds = gyroInputs.velocityAntiTipping;

            setpointSpeeds = antiTippingSpeeds;
        }

        // Convert the chassis speeds to swerve module states using the setpoint generator
        SwerveSetpoint previousSetpoint = moduleStateSetpoint;

        moduleStateSetpoint = setpointGenerator.generateSetpoint(
            moduleStateSetpoint,
            setpointSpeeds,
            Constants.LOOP_PERIOD_SECONDS
        );

        SwerveModuleState[] setpointStates = moduleStateSetpoint.moduleStates();
        DriveFeedforwards feedforwards = moduleStateSetpoint.feedforwards();

        double[] feedforwardLinearForcesNewtons = feedforwards.linearForcesNewtons();

        double[] angleMotorVelocitiesRadPerSec = new double[4];
        for (int i = 0; i < 4; i++) {
            angleMotorVelocitiesRadPerSec[i] =
                moduleStateSetpoint.moduleStates()[i].angle.getRadians() -
                previousSetpoint.moduleStates()[i].angle.getRadians();
            angleMotorVelocitiesRadPerSec[i] /= Constants.LOOP_PERIOD_SECONDS;
        }

        // Log setpoints
        Logger.recordOutput("Swerve/States/Setpoints", setpointStates);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", moduleStateSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("Swerve/ChassisSpeeds/SetpointsRaw", setpointSpeeds);
        Logger.recordOutput("Swerve/ChassisSpeeds/Accelerations", feedforwards.accelerationsMPSSq());

        Logger.recordOutput("Swerve/States/FeedforwardLinearForces", feedforwardLinearForcesNewtons);
        Logger.recordOutput("Swerve/States/FeedforwardTorqueCurrent", feedforwards.torqueCurrentsAmps());

        // Set the desired state for each swerve module
        setModuleStates(setpointStates, feedforwardLinearForcesNewtons, angleMotorVelocitiesRadPerSec);
    }

    /**
     * Stops the drive by running zero chassis speeds.
     */
    public void stop() {
        runVelocityChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * Sets the desired states for the swerve modules.
     * @param desiredStates - The desired states for the swerve modules.
     * @param feedforwardLinearForcesNewtons - The feedforward linear forces for each module in Newtons.
     * @param angleMotorVelocitiesRadPerSec - The desired angle motor velocities for each module in radians per second.
     */
    public void setModuleStates(
        SwerveModuleState[] desiredStates,
        double[] feedforwardLinearForcesNewtons,
        double[] angleMotorVelocitiesRadPerSec
    ) {
        // Set the desired state for each swerve module
        for (int i = 0; i < 4; i++) {
            Module mod = swerveModules[i];

            double driveFFVolts = swerveFeedForwards.getDriveMotorFFVolts(
                desiredStates[mod.index].speedMetersPerSecond / SwerveConstants.WHEEL_RADIUS.in(Meters),
                feedforwardLinearForcesNewtons[mod.index]
            );

            double angleFFVolts = swerveFeedForwards.getAngleMotorFFVolts(angleMotorVelocitiesRadPerSec[mod.index]);

            mod.runSetpoint(desiredStates[mod.index], driveFFVolts, angleFFVolts);
        }
    }

    /**
     * @return The current pose of the robot from the {@link #poseEstimator}.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * @return The actual pose of the robot. When in simulation mode, this will return the pose of the robot in the simulation world.
     * When in real mode, this will return the same as {@link #getPose}.
     */
    public Pose2d getActualPose() {
        return getPose();
    }

    /**
     * @return The current odometry rotation from {@link #getPose}
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Sets the pose of the robot in the pose estimator.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a new timestamped vision measurement.
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    private final SwerveModuleState[] cachedModuleStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
    };

    /**
     * @return The current module states.
     */
    @AutoLogOutput(key = "Swerve/States/Measured")
    public SwerveModuleState[] getModuleStates() {
        return cachedModuleStates;
    }

    private final SwerveModulePosition[] cachedModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };

    /**
     * @return The current module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        return cachedModulePositions;
    }

    private ChassisSpeeds cachedSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    /**
     * @return The measured chassis speeds of the robot.
     * Note: be careful when mutating the returned object, as it is cached.
     */
    @AutoLogOutput(key = "Swerve/ChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return cachedSpeeds;
    }

    private final MutLinearVelocity cachedVelocityMagnitude = MetersPerSecond.mutable(0);

    /**
     * @return The magnitude of the robot's velocity. Calculated from {@link #getChassisSpeeds()}.
     */
    @AutoLogOutput(key = "Swerve/ChassisSpeeds/Magnitude")
    public LinearVelocity getVelocityMagnitude() {
        return cachedVelocityMagnitude;
    }

    /**
     * @return The velocity magnitude of the robot in meters per second.
     */
    public double getVelocityMagnitudeAsDouble() {
        ChassisSpeeds speeds = getChassisSpeeds();

        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    private double[] cachedOrientationPublish = new double[6];

    /**
     * Returns orientation data for a Limelight.
     * See {@link frc.util.LimelightHelpers#SetRobotOrientation} for usage/format.
     * @return The orientation data to publish to the network.
     */
    public double[] getOrientationToPublish() {
        cachedOrientationPublish[0] = getRotation().getDegrees();
        cachedOrientationPublish[1] = gyroInputs.yawVelocity.in(DegreesPerSecond);
        // cachedOrientationPublish[2] = gyroInputs.pitchRadians.in(Degrees);
        // cachedOrientationPublish[3] = 0.0;
        // cachedOrientationPublish[4] = gyroInputs.rollRadians.in(Degrees);
        // cachedOrientationPublish[5] = 0.0;

        return cachedOrientationPublish;
    }

    /**
     * Zeros the yaw offset to the specified angle.
     * Note that this does not zero the gyro itself, but rather sets the offset used for field-oriented driving.
     * @param zeroRotation - The angle to zero the yaw offset to. This is added to the current robot heading.
     * Ex. when the robot is facing towards the driver station, this should be 180 degrees.
     */
    public void zeroYawOffset(Rotation2d zeroRotation) {
        yawOffset = getPose().getRotation().plus(zeroRotation);
    }

    /**
     * Zeros the yaw offset to 0 degrees.
     * Call when the robot is facing away from the driver station to have the robot oriented correctly on the field.
     */
    public void zeroYawOffset() {
        zeroYawOffset(Rotation2d.kZero);
    }

    /**
     * @return The robot heading for field-oriented driving, adjusted by the yaw offset.
     */
    public Rotation2d getHeadingForFieldOriented() {
        return getPose().getRotation().minus(yawOffset);
    }

    /**
     * @return The robot heading from the gyro, adjusted by the yaw offset.
     */
    public Rotation2d getHeadingFromGyro() {
        return rawGyroRotation.minus(yawOffset);
    }

    @Override
    public void periodic() {
        // Prevents odometry updates while reading data
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (Module module : swerveModules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Update cached values
        cachedSpeeds = kinematics.toChassisSpeeds(getModuleStates());
        cachedVelocityMagnitude.mut_replace(getVelocityMagnitudeAsDouble(), MetersPerSecond);

        for (int i = 0; i < 4; i++) {
            Module mod = swerveModules[i];
            cachedModuleStates[mod.index] = mod.getState();
            cachedModulePositions[i] = mod.getPosition();
        }

        // Init previous setpoint if null
        if (moduleStateSetpoint == null) {
            moduleStateSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            stop();
        }

        // Update odometry
        // All signals are sampled together; only need to get timestamps once
        double[] sampleTimestamps = gyroInputs.odometryYawTimestamps;
        int sampleCount = sampleTimestamps.length;

        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas;

            // Deltas not used when gyro is connected
            if (!gyroInputs.connected) {
                moduleDeltas = new SwerveModulePosition[4];
            } else {
                moduleDeltas = null;
            }

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[sampleIndex];

                // Only calculate module deltas if gyro is disconnected
                if (!gyroInputs.connected && moduleDeltas != null) {
                    moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle
                    );
                }

                // Update last positions
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[sampleIndex];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[sampleIndex], rawGyroRotation, modulePositions);
        }

        // Run state machine
        var currentState = stateMachine.getCurrentState().enumType;
        var action = stateMachine.statePeriodicActions.get(currentState);

        // TODO: better validation
        if (action != null && getCurrentCommand() == null) {
            action.onPeriodic(stateMachine.getCurrentPayload());
        }

        // Update aim to target
        if (
            // Payload must exist for target pose
            stateMachine.getCurrentPayload().isPresent() &&
            // Only run auto-aim if in auto aim state or if the payload requests rotation to pose
            (stateMachine.is(SwerveState.AUTO_AIM) ||
                stateMachine.getCurrentPayload().get().shouldRotateToPoseSupplier().get() !=
                SwervePayload.RotationMode.ONLY_DRIVE_NO_ROTATE)
        ) {
            Pose2d targetPoseToRotateTo = stateMachine.getCurrentPayload().get().poseToRotateToSupplier().get();

            // TODO: optimize and move to constants
            final int refinementSteps = 1;

            SwerveSetpoint moduleStateSetpointWithoutRotation = moduleStateSetpoint;

            for (int i = 0; i < refinementSteps; i++) {
                moduleStateSetpointWithoutRotation = setpointGenerator.generateSetpoint(
                    moduleStateSetpointWithoutRotation,
                    setpointSpeeds,
                    Constants.LOOP_PERIOD_SECONDS
                );

                Translation2d desiredChassisSpeedAcceleration = new Translation2d(
                    moduleStateSetpointWithoutRotation.robotRelativeSpeeds().vxMetersPerSecond -
                    moduleStateSetpoint.robotRelativeSpeeds().vxMetersPerSecond,
                    moduleStateSetpointWithoutRotation.robotRelativeSpeeds().vyMetersPerSecond -
                    moduleStateSetpoint.robotRelativeSpeeds().vyMetersPerSecond
                ).div(0.02);

                autoAim.updateCalculatedResult(
                    getPose(),
                    targetPoseToRotateTo,
                    moduleStateSetpointWithoutRotation.robotRelativeSpeeds(),
                    getChassisSpeeds(),
                    desiredChassisSpeedAcceleration,
                    setpointSpeeds
                );

                // Override the angular velocity setpoint with the auto-aim result
                setpointSpeeds.omegaRadiansPerSecond = autoAim.getRotationOutputRadiansPerSecond();
            }

            autoAim.latestCalculationResult.log();

            // debug
            Logger.recordOutput(
                "AimToTarget/RotationErrorRad",
                getRotation().minus(new Rotation2d(autoAim.latestCalculationResult.getRotationTarget())).getRadians()
            );
        }

        // Apply outputs
        if (shouldRunSpeedsThisCycle) {
            runSpeeds();
            shouldRunSpeedsThisCycle = false;
        }
    }

    // Characterization methods

    /**
     * Returns a command to run the max acceleration / max velocity test. Runs the drive in a straight line at maximum voltage.
     * Use AdvantageScope line graph to analyze the results.
     * By default, this does nothing.
     */
    public Command runMaxAccelerationMaxVelocityTest() {
        return Commands.run(() -> runCharacterization(12.0), this);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     * @param output - The output voltage to apply to the drive motors.
     */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runCharacterization(output);
        }
    }

    /**
     * Runs the angle motors with the specified output voltage.
     * @param output - The output voltage to apply to the angle motors.
     */
    public void runAngleCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runAngleCharacterization(output);
        }
    }

    /**
     * Returns a command to run a drive quasistatic test in the specified direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(driveSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a drive dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(driveSysId.dynamic(direction));
    }

    /**
     * Returns a command to run an angle quasistatic test in the specified direction.
     */
    public Command angleSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runAngleCharacterization(0.0)).withTimeout(1.0).andThen(angleSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run an angle dynamic test in the specified direction.
     */
    public Command angleSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runAngleCharacterization(0.0)).withTimeout(1.0).andThen(angleSysId.dynamic(direction));
    }

    /**
     * @return The position of each module's drive in radians.
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = swerveModules[i].getWheelRadiusCharacterizationPosition().in(Radians);
        }
        return values;
    }

    /**
     * @return The average velocity of the modules in rad/sec.
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += swerveModules[i].getFFCharacterizationVelocity().in(RadiansPerSecond) / 4.0;
        }
        return output;
    }

    /**
     * @return The wheel slipping characterization current in Amps, if any module is slipping.
     */
    public OptionalDouble getWheelSlippingCharacterization() {
        for (int i = 0; i < 4; i++) {
            // Check if the module is slipping by seeing if the velocity is nonzero
            if (swerveModules[i].getFFCharacterizationVelocity().in(RadiansPerSecond) < 0.175) continue;

            return OptionalDouble.of(swerveModules[i].getWheelSlippingCharacterization().in(Amps));
        }

        return OptionalDouble.empty();
    }
}
