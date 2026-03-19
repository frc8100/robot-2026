package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ButtonBindings;
import frc.robot.CANIdConstants;
import frc.robot.Constants;
import frc.robot.commands.AimToTarget;
import frc.robot.subsystems.DeviceAlert;
import frc.robot.subsystems.SparkAlert;
import frc.robot.subsystems.intake.Intake.IntakeState;
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
    public static double getCurrentPredictedFuelExitVelocityFromMotor(
        ShotCalculator shotCalculator,
        AngularVelocity motorAngularVelocity
    ) {
        // double predictedDistanceMeters = motorAngularVelocityToDistanceMap.get(
        //     motorAngularVelocity.in(RadiansPerSecond)
        // );

        // return AimToTarget.distanceToExitVelocityMap.get(predictedDistanceMeters);

        return (
            shotCalculator.getRpmMap().getInverseMap().get(motorAngularVelocity.in(RPM)) /
            (shotCalculator.getTofMap().getInverseMap().get(motorAngularVelocity.in(RPM)))
        );
    }

    // Inverse maps
    // private static final InterpolatingDoubleTreeMap motorAngularVelocityToDistanceMap =
    //     ShooterConstants.distanceToMotorAngularVelocityMap.getInverseMap();

    public enum ShooterSpeeds {
        LOW(ShooterConstants.SHOOTER_MAX_SPEED.times(0.25)),
        MEDIUM(ShooterConstants.SHOOTER_MAX_SPEED.times(0.5)),
        HIGH(ShooterConstants.SHOOTER_MAX_SPEED.times(0.75)),
        HIGHEST(ShooterConstants.SHOOTER_MAX_SPEED);

        public final AngularVelocity speed;

        private ShooterSpeeds(AngularVelocity speed) {
            this.speed = speed;
        }
    }

    public enum ShooterState {
        /**
         * Not shooting.
         */
        IDLE,

        /**
         * The shooter is spinning to the target velocity and the indexer is running to move fuel to the shooter.
         * The target velocity is determined automatically by the distance to the target.
         */
        AUTO_TARGET_SHOOTING,

        /**
         * The shooter is spinning to the target velocity and the indexer is running to move fuel to the shooter.
         * Target velocity is determined manually.
         */
        MANUAL_TARGET_SHOOTING,

        /**
         * The intake deploy is running duty cycle based on the voltage controlled by the human operator.
         */
        TEST_VOLTAGE_CONTROL,
    }

    public final StateMachine<ShooterState, Object> stateMachine = new StateMachine<ShooterState, Object>(
        ShooterState.class,
        "Shooter"
    )
        .withDefaultState(new StateMachineState<>(ShooterState.IDLE, "Idle"))
        .withState(new StateMachineState<>(ShooterState.AUTO_TARGET_SHOOTING, "AutoShooting"))
        .withState(new StateMachineState<>(ShooterState.MANUAL_TARGET_SHOOTING, "ManualShooting"))
        .withState(new StateMachineState<>(ShooterState.TEST_VOLTAGE_CONTROL, "TestVoltage"))
        .withReturnToDefaultStateOnDisable(true);

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Subsystem references
    private final Swerve swerveSubsystem;

    // Caches
    private final MutAngularVelocity cachedTargetExitAngularVelocity = RadiansPerSecond.mutable(0.0);

    private final MutVoltage testVoltageOutputShooter = Volts.mutable(0.0);
    private final MutVoltage testVoltageOutputIndexer = Volts.mutable(0.0);
    private final SysIdRoutine shooterSysidRoutine;

    // Alerts for disconnected motors
    private final DeviceAlert indexerDisconnectedAlert = new SparkAlert(
        () -> inputs.indexerMotorData,
        CANIdConstants.INDEXER_MOTOR_ID,
        "IndexShootMotor"
    );
    private final DeviceAlert leaderDisconnectedAlert = new SparkAlert(
        () -> inputs.leaderShootMotorData,
        CANIdConstants.LEFT_SHOOTER_MOTOR_ID,
        "LeftShootMotor"
    );
    private final DeviceAlert followerDisconnectedAlert = new SparkAlert(
        () -> inputs.followerShootMotorData,
        CANIdConstants.RIGHT_SHOOTER_MOTOR_ID,
        "RightShootMotor"
    );
    private final DeviceAlert beamBreakerDisconnectedAlert = new DeviceAlert("ShootBeamBreaker");

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
        stateMachine.whileState(ShooterState.AUTO_TARGET_SHOOTING, this::handleShootState);
        stateMachine.whileState(ShooterState.MANUAL_TARGET_SHOOTING, this::handleManualShootState);
        stateMachine.whileState(ShooterState.TEST_VOLTAGE_CONTROL, () -> {
            io.runShooterDutyCycle(testVoltageOutputShooter);
            io.runIndexerDutyCycle(testVoltageOutputIndexer);
        });

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

    public void changeTestOutVoltageShooter(Voltage change) {
        testVoltageOutputShooter.mut_plus(change);
    }

    public void setTestOutVoltageShooter(Voltage set) {
        testVoltageOutputShooter.mut_replace(set);
    }

    public void changeTestOutVoltageIndexer(Voltage change) {
        testVoltageOutputIndexer.mut_plus(change);
    }

    public void setTestOutVoltageIndexer(Voltage set) {
        testVoltageOutputIndexer.mut_replace(set);
    }

    public Command runShooterDutyCycle(Voltage dutyCycleOutput) {
        return run(() -> io.runShooterDutyCycle(dutyCycleOutput));
    }

    public Command changeShooterRPMOffset(double rpmOffset) {
        return Commands.runOnce(() -> swerveSubsystem.autoAim.shotCalculator.adjustOffset(rpmOffset));
    }

    /**
     * Handles the logic for the {@link ShooterState#IDLE} state which stops the shooter and indexer.
     */
    private void handleIdleState() {
        io.stopShooter();
        io.stopIndexer();
    }

    /**
     * Handles the logic for the {@link ShooterState#AUTO_TARGET_SHOOTING} state which sets the target exit velocity based on the distance to the target.
     */
    private void handleShootState() {
        setTargetExitVelocityToTarget();
        runIndexerIfShooterAtSpeed();
    }

    public Command setspeedShoot(ShooterSpeeds shooterSpeed) {
        return new Command() {
            @Override
            public void execute() {
                stateMachine.scheduleStateChange(ShooterState.MANUAL_TARGET_SHOOTING);
                setTargetExitVelocity(shooterSpeed.speed);
                // io.runShooterDutyCycle(Volts.of(12));
            }

            // @Override
            // public void execute() {
            //     io.runShooterDutyCycle(Volts.of(12));
            // }

            @Override
            public void end(boolean interrupted) {
                stateMachine.scheduleStateChange(ShooterState.IDLE);
            }
        };
        // return run(() -> io.runShooterDutyCycle(Volts.of(12)));
    }

    private void handleManualShootState() {
        boolean usingManualControls = ButtonBindings.operatorController.getRawButton(
            XboxController.Button.kRightBumper.value
        );

        double controllerAngle = Math.atan2(
            -ButtonBindings.operatorController.getRawAxis(XboxController.Axis.kLeftY.value),
            ButtonBindings.operatorController.getRawAxis(XboxController.Axis.kLeftX.value)
        );

        if (controllerAngle < 0) {
            controllerAngle += 2 * Math.PI;
        }

        Logger.recordOutput("Shooter/ControllerAngle", controllerAngle);

        if (usingManualControls) {
            double controllerMagnitude = controllerAngle / (Math.PI * 2);
            AngularVelocity targetVelocity = ShooterConstants.SHOOTER_MAX_SPEED.times(controllerMagnitude);
            setTargetExitVelocity(targetVelocity);
        } else {
            setTargetExitVelocity(cachedTargetExitAngularVelocity);
        }

        runIndexerIfShooterAtSpeed();
    }

    public void runIndexerIfShooterAtSpeed() {
        boolean shooterUpToSpeed = inputs.leaderShootMotorData.velocity.isNear(
            cachedTargetExitAngularVelocity,
            RadiansPerSecond.of(22.0)
        );

        Logger.recordOutput("Shooter/UpToSpeed", shooterUpToSpeed);

        // TODO: add override
        boolean isBeingOverrided = ButtonBindings.operatorController.getRawButton(
            XboxController.Button.kRightBumper.value
        );
        if (shooterUpToSpeed || isBeingOverrided) {
            io.setIndexerVelocitySetpoint(ShooterConstants.INDEXER_SPEED);
            // io.runIndexerDutyCycle(Volts.of(6));
        } else {
            io.stopIndexer();
        }
    }

    public Command runIndexer() {
        // return Commands.run(() -> io.setIndexerVelocitySetpoint(ShooterConstants.INDEXER_SPEED)).andThen(
        //     io::stopIndexer
        // );
        return Commands.run(() -> io.runIndexerDutyCycle(Volts.of(3))).andThen(io::stopIndexer);
    }

    /**
     * Predicts the trajectory of a fuel based on the current shooter exit velocity, angle, and robot velocity. Also predicts whether the fuel would score in the hub.
     */
    // private void updatePredictedFuelTrajectory(
    //     CachedPredictedTrajectoryResult result,
    //     double exitVelocityMPS,
    //     boolean shouldLogExtraData
    // ) {
    //     result.trajectoryPoints.clear();

    //     // Precompute constants
    //     final Rotation2d shooterAngle = swerveSubsystem
    //         .getPose()
    //         .getRotation()
    //         .plus(ShooterConstants.AIM_ROTATION_OFFSET);
    //     final ChassisSpeeds robotVelocity = swerveSubsystem.getFieldRelativeSpeeds();

    //     // Calculate the initial velocity vector based on the exit velocity and angle
    //     Translation3d velocity = new Translation3d(
    //         exitVelocityMPS * ShooterConstants.exitAngle.getCos() * shooterAngle.getCos() +
    //         robotVelocity.vxMetersPerSecond,
    //         exitVelocityMPS * ShooterConstants.exitAngle.getCos() * shooterAngle.getSin() +
    //         robotVelocity.vyMetersPerSecond,
    //         exitVelocityMPS * ShooterConstants.exitAngle.getSin()
    //     );

    //     Translation3d position = new Pose3d(swerveSubsystem.getPose())
    //         .transformBy(ShooterConstants.transformFromRobotCenter)
    //         .getTranslation();

    //     // Predict final position at time to target
    //     double finalT =
    //         swerveSubsystem.autoAim.latestCalculationResult.getTimeToTarget().in(Seconds) +
    //         ShooterConstants.PREDICT_FUEL_POSITION_LOOKAHEAD_TIME.in(Seconds);
    //     Translation3d finalPosition = new Translation3d(
    //         position.getX() + velocity.getX() * finalT,
    //         position.getY() + velocity.getY() * finalT,
    //         position.getZ() +
    //         velocity.getZ() * finalT -
    //         0.5 * ShooterConstants.g.in(MetersPerSecondPerSecond) * finalT * finalT
    //     );
    //     Translation3d velocityAtFinalPosition = new Translation3d(
    //         velocity.getX(),
    //         velocity.getY(),
    //         velocity.getZ() - ShooterConstants.g.in(MetersPerSecondPerSecond) * finalT
    //     );

    //     // Check if the predicted final position would score in the hub
    //     FuelSim.Fuel predictedFuel = new FuelSim.Fuel(finalPosition, velocityAtFinalPosition);
    //     boolean hitTarget =
    //         // Check both hubs because we aren't aiming at the opponent's hub (hopefully)
    //         FuelSim.Hub.BLUE_HUB.didFuelScoreAtAll(predictedFuel) ||
    //         FuelSim.Hub.RED_HUB.didFuelScoreAtAll(predictedFuel);

    //     result.finalPoint = finalPosition;
    //     result.hitTarget = hitTarget;

    //     // If we don't need to log additional data, skip calculating the full trajectory for performance reasons
    //     if (!shouldLogExtraData) {
    //         result.finalPoint = finalPosition;
    //         result.hitTarget = hitTarget;
    //     }

    //     for (int i = 0; i < 30; i++) {
    //         double t = i * 0.1;

    //         // If we've reached the final time, add the final position and break
    //         if (t > finalT) {
    //             result.trajectoryPoints.add(finalPosition);

    //             break;
    //         }

    //         // Calculate with simple projectile motion equations with gravity
    //         double z =
    //             position.getZ() + velocity.getZ() * t - 0.5 * ShooterConstants.g.in(MetersPerSecondPerSecond) * t * t;

    //         // If below ground level, stop the trajectory
    //         if (z < 0) {
    //             result.trajectoryPoints.add(
    //                 new Translation3d(position.getX() + velocity.getX() * t, position.getY() + velocity.getY() * t, 0)
    //             );
    //             break;
    //         }

    //         double x = position.getX() + velocity.getX() * t;
    //         double y = position.getY() + velocity.getY() * t;

    //         // Update position for the next iteration
    //         result.trajectoryPoints.add(new Translation3d(x, y, z));
    //     }
    // }

    /**
     * Sets the target exit velocity for the shooter. Does not look up.
     * @param targetExitVelocity - The target exit velocity in radians per second of the shooter motor angular velocity.
     */
    public void setTargetExitVelocity(AngularVelocity targetExitVelocity) {
        cachedTargetExitAngularVelocity.mut_replace(targetExitVelocity);
        io.setTargetShootMotorVelocity(targetExitVelocity);
    }

    /**
     * Sets the target exit velocity for the shooter based on the distance to the target.
     * @param distanceToTarget - The distance to the target in meters.
     */
    public void setTargetExitVelocityToTarget() {
        // Look up the corresponding motor angular velocity for the given distance and update cache
        cachedTargetExitAngularVelocity.mut_replace(
            // ShooterConstants.distanceToMotorAngularVelocityMap.get(distanceToTarget.in(Meters)),
            swerveSubsystem.autoAim.latestCalculationResult.result.rpm(),
            RPM
        );

        io.setTargetShootMotorVelocity(cachedTargetExitAngularVelocity);
    }

    public void setTargetExitVelocityToTarget(AngularVelocity targetExitVelocity) {
        cachedTargetExitAngularVelocity.mut_replace(targetExitVelocity);
        io.setTargetShootMotorVelocity(cachedTargetExitAngularVelocity);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Update alerts
        indexerDisconnectedAlert.updateConnectionStatus(inputs.indexerMotorConnected);
        leaderDisconnectedAlert.updateConnectionStatus(inputs.leaderShootMotorConnected);
        followerDisconnectedAlert.updateConnectionStatus(inputs.followerShootMotorConnected);
        beamBreakerDisconnectedAlert.updateConnectionStatus(inputs.beamBreakerConnected);
        // Log trajectory points for visualization
        // updatePredictedFuelTrajectory(
        //     cachedTargetTrajectoryResult,
        //     // swerveSubsystem.autoAim.latestCalculationResult.getTargetFuelExitVelocity().in(MetersPerSecond),
        //     // swerveSubsystem.autoAim.shotCalculator.effectiveRPM(0)
        //     Constants.shouldLogAdditionalData()
        // );
        // cachedTargetTrajectoryResult.log("Shooter/TargetTrajectory");

        // updatePredictedFuelTrajectory(
        //     cachedCurrentTrajectoryResult,
        //     getCurrentPredictedFuelExitVelocityFromMotor(inputs.leaderShootMotorData.velocity),
        //     Constants.shouldLogAdditionalData()
        // );
        // cachedCurrentTrajectoryResult.log("Shooter/CurrentTrajectory");

        // updatePredictedFuelTrajectory(
        //     cachedFutureTrajectoryResult,
        //     getCurrentPredictedFuelExitVelocityFromMotor(inputs.leaderShootMotorData.velocity) + 0.2,
        //     Constants.shouldLogAdditionalData()
        // );
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
        return inputs.leaderShootMotorData.velocity;
    }
}
