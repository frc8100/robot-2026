package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ControlConstants;
import frc.robot.commands.AimToTarget;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.swerve.Swerve.SwervePayload;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.util.SwerveFeedForwards;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachine.StatePeriodicAction;
import frc.util.statemachine.StateMachineState;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Swerve subsystem, responsible for controlling the swerve drive. */
public class Swerve extends SubsystemBase implements SwerveDrive {

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
        Supplier<Pose2d> poseSupplier,
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
            return new SwervePayload(poseSupplier, () -> RotationMode.ONLY_DRIVE_NO_ROTATE, () -> Pose2d.kZero);
        }
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

    /**
     * A representation of the field for visualization (on Elastic).
     * Currently disabled to increase performance.
     * (use AdvantageScope for field visualization instead)
     */
    // protected Field2d field = new Field2d();

    public final SwerveDrivePoseEstimator poseEstimator;

    /**
     * The yaw offset for field-oriented driving.
     */
    private Rotation2d yawOffset = Rotation2d.kZero;

    protected SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, state -> Logger.recordOutput("Swerve/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(voltage -> runCharacterization(voltage.in(Volt)), null, this)
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

        zeroGyro(180);

        // Set up custom logging to add the current path to a field 2d widget
        // PathPlannerLogging.setLogActivePathCallback(poses -> field.getObject("path").setPoses(poses));
        // SmartDashboard.putData("Field", field);

        teleopSwerve = new TeleopSwerve(
            this,
            // Switch between joystick and main drive controls depending on the mode
            ControlConstants.isUsingJoystickDrive
                ? ControlConstants.joystickDriveControls
                : ControlConstants.mainDriveControls,
            true
        );

        // Sync motor encoders to absolute encoders when the robot is still
        syncMotorEncodersToAbsoluteEncoderTrigger.onTrue(
            Commands.runOnce(() -> {
                for (Module module : swerveModules) {
                    module.syncMotorEncoderToAbsoluteEncoder();
                }

                // debug
                System.out.println("Swerve motor encoders synced");
            })
        );

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

    @Override
    public void drive(double xMeters, double yMeters, double omegaRadPerSec, boolean fieldRelative) {
        ChassisSpeeds desiredChassisSpeeds = getSpeedsFromTranslation(xMeters, yMeters, omegaRadPerSec, fieldRelative);

        runVelocityChassisSpeeds(desiredChassisSpeeds);
    }

    @Override
    public void runVelocityChassisSpeeds(ChassisSpeeds speed) {
        setpointSpeeds = speed;
    }

    @SuppressWarnings("unused")
    public void runSpeeds() {
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
        moduleStateSetpoint = setpointGenerator.generateSetpoint(
            moduleStateSetpoint,
            setpointSpeeds,
            Constants.LOOP_PERIOD_SECONDS
        );

        SwerveModuleState[] setpointStates = moduleStateSetpoint.moduleStates();
        DriveFeedforwards feedforwards = moduleStateSetpoint.feedforwards();

        double[] feedforwardLinearForcesNewtons = feedforwards.linearForcesNewtons();

        // Log setpoints
        Logger.recordOutput("Swerve/States/Setpoints", setpointStates);
        Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", moduleStateSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("Swerve/ChassisSpeeds/SetpointsRaw", setpointSpeeds);
        Logger.recordOutput("Swerve/ChassisSpeeds/Accelerations", feedforwards.accelerationsMPSSq());

        Logger.recordOutput("Swerve/States/FeedforwardLinearForces", feedforwardLinearForcesNewtons);
        Logger.recordOutput("Swerve/States/FeedforwardTorqueCurrent", feedforwards.torqueCurrentsAmps());

        // Set the desired state for each swerve module
        setModuleStates(setpointStates, feedforwardLinearForcesNewtons);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, double[] feedforwardLinearForcesNewtons) {
        // Set the desired state for each swerve module
        for (int i = 0; i < 4; i++) {
            Module mod = swerveModules[i];

            double driveFFVolts = swerveFeedForwards.getLinearForceFFVolts(
                desiredStates[mod.index].speedMetersPerSecond / SwerveConstants.WHEEL_RADIUS.in(Meters),
                feedforwardLinearForcesNewtons[mod.index]
            );

            mod.runSetpoint(desiredStates[mod.index], driveFFVolts);
        }
    }

    @Override
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runCharacterization(output);
        }
    }

    @Override
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    @Override
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    @Override
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = swerveModules[i].getWheelRadiusCharacterizationPosition().in(Radians);
        }
        return values;
    }

    @Override
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += swerveModules[i].getFFCharacterizationVelocity().in(RadiansPerSecond) / 4.0;
        }
        return output;
    }

    public Optional<Double> getWheelSlippingCharacterization() {
        for (int i = 0; i < 4; i++) {
            // Check if the module is slipping by seeing if the velocity is nonzero
            if (swerveModules[i].getFFCharacterizationVelocity().in(RadiansPerSecond) < 0.175) continue;

            return Optional.of(swerveModules[i].getWheelSlippingCharacterization().in(Amps));
        }

        return Optional.empty();
    }

    @Override
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    @Override
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

    @Override
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

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return cachedModulePositions;
    }

    private ChassisSpeeds cachedSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    @Override
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

    @Override
    public void zeroGyro(double deg) {
        yawOffset = getPose().getRotation().plus(Rotation2d.fromDegrees(deg));
    }

    @Override
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

        if (action != null) {
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
        runSpeeds();
    }
}
