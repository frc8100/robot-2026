package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.util.SwerveFeedForwards;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Contains more complex characterization routines.
 */
public class SwerveSysidRoutines {

    private SwerveSysidRoutines() {}

    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static final RobotConfig PP_CONFIG = SwerveConstants.getRobotConfig();

    private static ChassisSpeeds previousSpeeds = new ChassisSpeeds();

    private static record FFMeasurement(double chassisAccelMPS2, double linearForcesNewtons) {}

    /**
     * A functional interface that supplies a voltage based on the elapsed time in seconds.
     */
    @FunctionalInterface
    private interface VoltageSupplierFromTime {
        double getVoltage(double timeSeconds);
    }

    /**
     * Calculates the supplied linear forces in Newtons for the given chassis speeds.
     * Based on PathPlanner's SwerveSetpointGenerator implementation.
     * @param newSpeeds - The current chassis speeds.
     * @return The calculated wheel forces in Newtons.
     */
    private static FFMeasurement getLinearForcesNewtons(ChassisSpeeds newSpeeds) {
        double chassisAccelX =
            (newSpeeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond) / Constants.LOOP_PERIOD_SECONDS;
        double chassisAccelY =
            (newSpeeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond) / Constants.LOOP_PERIOD_SECONDS;
        double chassisForceX = chassisAccelX * PP_CONFIG.massKG;
        double chassisForceY = chassisAccelY * PP_CONFIG.massKG;

        // Angular force should be 0 because driving straight
        double angularAccel =
            (newSpeeds.omegaRadiansPerSecond - previousSpeeds.omegaRadiansPerSecond) / Constants.LOOP_PERIOD_SECONDS;
        double angTorque = angularAccel * PP_CONFIG.MOI;

        ChassisSpeeds chassisForces = new ChassisSpeeds(chassisForceX, chassisForceY, angTorque);

        Translation2d[] wheelForces = PP_CONFIG.chassisForcesToWheelForceVectors(chassisForces);

        double wheelForceDist = wheelForces[0].getNorm();

        // Update previous speeds
        previousSpeeds = newSpeeds;

        return new FFMeasurement(Math.hypot(chassisAccelX, chassisAccelY), wheelForceDist);
    }

    /**
     * @return A command that runs a single feedforward characterization test with the given voltage supplier.
     */
    private static Command feedforwardCharacterizationRunOnce(
        SwerveDrive drive,
        VoltageSupplierFromTime voltageSupplier
    ) {
        Timer timer = new Timer();

        return Commands.sequence(
            // Reorient modules to face forward
            getFeedforwardCharacterizationReorientCommand(drive),
            // Start timer
            Commands.runOnce(timer::restart),
            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = voltageSupplier.getVoltage(timer.get());
                    drive.runCharacterization(voltage);

                    double currentVelocityRadPerSec = drive.getFFCharacterizationVelocity();
                    double currentVelocityMPS = currentVelocityRadPerSec * SwerveConstants.WHEEL_RADIUS.in(Meters);

                    ChassisSpeeds currentSpeeds = new ChassisSpeeds(currentVelocityMPS, 0.0, 0.0);

                    var measurement = getLinearForcesNewtons(currentSpeeds);

                    Logger.recordOutput("SysId/FFCharacterization/VelocityRadPerSec", currentVelocityRadPerSec);
                    Logger.recordOutput("SysId/FFCharacterization/VelocitySign", Math.signum(currentVelocityRadPerSec));
                    Logger.recordOutput(
                        "SysId/FFCharacterization/LinearForcesFFVolts",
                        SwerveFeedForwards.getLinearForcesFFVoltsFromRadPerSec(
                            measurement.linearForcesNewtons,
                            currentVelocityRadPerSec
                        )
                    );
                    Logger.recordOutput("SysId/FFCharacterization/ChassisAccelMPS2", measurement.chassisAccelMPS2);
                    Logger.recordOutput("SysId/FFCharacterization/AppliedVoltage", voltage);
                },
                drive
            ).andThen(
                Commands.runOnce(() -> {
                    drive.runCharacterization(0.0);
                    previousSpeeds = new ChassisSpeeds();
                })
            )
        );
    }

    private static Command getFeedforwardCharacterizationReorientCommand(SwerveDrive drive) {
        return Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY);
    }

    /**
     * Measures data for feedforward characterization.
     * Export as csv using AdvantageScope and the prefix `/RealOutputs/SysId/FFCharacterization,DS:enabled`
     * and analyze using the python script at `regression/drive_ff_characterization.py`.
     */
    public static Command feedforwardCharacterization(SwerveDrive drive) {
        return Commands.sequence(
            // Tests with voltage ramping
            feedforwardCharacterizationRunOnce(drive, (double seconds) -> seconds * 1.0).withTimeout(Seconds.of(5)),
            feedforwardCharacterizationRunOnce(drive, (double seconds) -> seconds * -1.0).withTimeout(Seconds.of(5)),
            // Tests with constant voltage
            feedforwardCharacterizationRunOnce(drive, (double seconds) -> Math.min(7.0, 7.0 * seconds)).withTimeout(
                Seconds.of(3)
            ),
            feedforwardCharacterizationRunOnce(drive, (double seconds) -> Math.max(-7.0, -7.0 * seconds)).withTimeout(
                Seconds.of(3)
            )
        );
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Swerve drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),
                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                        drive.runVelocityChassisSpeeds(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive
                )
            ),
            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),
                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drive.getHeadingFromGyro();
                    state.gyroDelta = 0.0;
                }),
                // Update gyro delta
                Commands.run(() -> {
                    var rotation = drive.getHeadingFromGyro();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                }).finallyDo(() -> { // When cancelled, calculate and print results
                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius = (state.gyroDelta * SwerveConstants.DRIVE_BASE_RADIUS.in(Meters)) / wheelDelta;

                    NumberFormat formatter = new DecimalFormat("#0.000");
                    System.out.println("********** Wheel Radius Characterization Results **********");
                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                    System.out.println(
                        "\tWheel Radius: " +
                        formatter.format(wheelRadius) +
                        " meters, " +
                        formatter.format(Units.metersToInches(wheelRadius)) +
                        " inches"
                    );
                })
            )
        );
    }

    private static class WheelRadiusCharacterizationState {

        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    /**
     * Drives the robot until the wheels slip. The robot should be facing a wall.
     * From https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/docs/getting-started/template-projects/spark-swerve-template.md:
     * 1. Place the robot against the solid wall.
     * 2. Using AdvantageScope, plot the current of a drive motor from the /Drive/Module.../DriveCurrentAmps key, and the velocity of the motor from the /Drive/Module.../DriveVelocityRadPerSec key.
     * 3. Accelerate forward until the drive velocity increases (the wheel slips). Note the current at this time.
     * 4. Update the value of driveMotorCurrentLimit to this value.
     */
    public static Command wheelSlipCurrentCharacterization(Swerve drive) {
        return Commands.run(() -> drive.runCharacterization(5), drive)
            .until(() -> drive.getWheelSlippingCharacterization().isPresent())
            .andThen(() -> {
                double slipData = drive.getWheelSlippingCharacterization().get();
                NumberFormat formatter = new DecimalFormat("#0.000");
                System.out.println("********** Wheel Slip Current Characterization Results **********");
                System.out.println("\tSlip Current: " + formatter.format(slipData) + " Amps");
            })
            .withTimeout(Seconds.of(10));
    }
}
