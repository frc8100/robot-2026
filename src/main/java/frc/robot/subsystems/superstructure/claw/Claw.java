package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The subsystem responsible for the arm.
 */
public class Claw extends SubsystemBase {

    /**
     * The IO interface for the claw.
     */
    // TODO: protect
    public final ClawIO io;

    /**
     * The inputs for the claw.
     */
    protected final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    /**
     * Creates a new Claw subsystem.
     * @param io - The IO interface for the claw.
     */
    public Claw(ClawIO io) {
        this.io = io;
    }

    public boolean isCoralInClaw() {
        return inputs.isCoralInClaw;
    }

    /**
     * @return Whether the claw is at the target angle, within a tolerance.
     */
    public boolean isClawAtTarget() {
        return MathUtil.isNear(
            inputs.turnSetpointRad,
            inputs.turnPositionRad,
            ClawConstants.ANGLE_TOLERANCE_RADIANS.in(Radians)
        );
    }

    /**
     * @return Whether the claw is at the given angle, within a tolerance.
     */
    public boolean isClawAtTarget(Rotation2d targetAngle) {
        return MathUtil.isNear(
            targetAngle.getRadians(),
            inputs.turnPositionRad,
            ClawConstants.ANGLE_TOLERANCE_RADIANS.in(Radians)
        );
    }

    public Command getRunAndAngleCommand(DoubleSupplier motorInputSupplier, DoubleSupplier angleInputSupplier) {
        return new RunCommand(
            () -> {
                io.runOuttake(motorInputSupplier.getAsDouble());
                io.increaseTurnPosition(
                    MathUtil.applyDeadband(angleInputSupplier.getAsDouble(), ClawConstants.CONTROLLER_DEADBAND) *
                    ControlConstants.Claw.clawIncreaseAmount
                );
            },
            this
        );
    }

    /**
     * @return A command to run the claw.
     * @param motorInputSupplier - the supplier for the percent motor input.
     */
    public Command getRunCommand(DoubleSupplier motorInputSupplier) {
        return new RunCommand(() -> io.runOuttake(motorInputSupplier.getAsDouble()), this);
    }

    /**
     * @return A command to move the claw to a specific angle.
     * @param rotationToRotateClawTo - the angle to move to
     */
    public Command getAngleCommand(Rotation2d rotationToRotateClawTo) {
        return new InstantCommand(() -> io.setTurnPosition(rotationToRotateClawTo), this);
    }

    /**
     * @return A command that sets the claw to a specific angle and waits until it is at that angle.
     * It will not finish until the claw is at the target angle.
     */
    public Command getWaitForAngleCommand(Rotation2d rotationToRotateClawTo) {
        // TODO: add constants
        // If the claw angle is 0, set it to an amount slightly above zero then set it to zero
        if (rotationToRotateClawTo.getDegrees() < 7) {
            // @formatter:off
            return (
                getAngleCommand(Rotation2d.fromDegrees(7))
                    .andThen(Commands.waitUntil(() -> isClawAtTarget(Rotation2d.fromDegrees(7))))
                    .andThen(getAngleCommand(rotationToRotateClawTo))
                    .andThen(Commands.waitUntil(() -> isClawAtTarget(rotationToRotateClawTo)))

            );
            // @formatter:on
        }

        // @formatter:off
        return (
            // First, set the target angle
            getAngleCommand(rotationToRotateClawTo)
                // Then, wait until the claw is at the target angle
                .andThen(Commands.waitUntil(() -> isClawAtTarget(rotationToRotateClawTo)))
        );
        // @formatter:on
    }

    /**
     * @return A command to run the claw intake or outtake for {@link ClawConstants.INTAKE_OUTTAKE_TIME}.
     * @param direction - The direction to run the intake or outtake. See {@link ClawConstants.IntakeOuttakeDirection} for the possible directions.
     * @param timeout - The amount of time to run the intake or outtake for.
     */
    public Command runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection direction, Time timeout) {
        return Commands.deadline(
            // Stop after a certain amount of time
            Commands.waitTime(timeout),
            // Run the outtake
            new RunCommand(() -> io.runOuttake(direction.getDirection()))
        ).finallyDo(
            // Stop the outtake
            // new InstantCommand(() -> io.runOuttake(0))
            () -> io.runOuttake(0)
        );
    }

    public Command runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection direction) {
        return runIntakeOrOuttake(
            direction,
            direction == ClawConstants.IntakeOuttakeDirection.BACK
                ? ClawConstants.INTAKE_TIME
                : ClawConstants.OUTTAKE_TIME
        );
    }

    public Command runIntakeUntilCoralIsInClaw(Time timeout) {
        return runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE, timeout)
            .until(() -> inputs.isCoralInClaw)
            // Decelerate
            .andThen(runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.BACK, Seconds.of(0.09)));
    }

    public Command runIntakeUntilCoralIsInClaw() {
        return runIntakeUntilCoralIsInClaw(ClawConstants.INTAKE_TIME);
    }

    public Command runOuttakeUntilCoralIsNotInClaw(Time timeout) {
        return runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE, timeout.div(2)).andThen(
            runIntakeOrOuttake(ClawConstants.IntakeOuttakeDirection.OUTTAKE, timeout).until(() -> !inputs.isCoralInClaw)
        );
    }

    public Command runOuttakeUntilCoralIsNotInClaw() {
        return runOuttakeUntilCoralIsNotInClaw(ClawConstants.OUTTAKE_TIME);
    }

    @Override
    public void periodic() {
        io.periodic();

        // Update inputs
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);
    }

    /**
     * @return The position of the claw.
     */
    public Pose3d getPose(Pose3d elevatorPose) {
        return new Pose3d(
            ClawConstants.RotationPositions.ELEVATOR_TO_CLAW_X,
            0,
            ClawConstants.RotationPositions.ELEVATOR_TO_CLAW_Z + elevatorPose.getZ(),
            new Rotation3d(0, inputs.turnPositionRad, 0)
        );
    }

    public Pose3d getPose() {
        return getPose(new Pose3d());
    }

    /**
     * @return The pose of the coral in the claw.
     * If the claw is not holding coral, return an empty array.
     */
    public Pose3d[] getCoralInClawPosition(SwerveDrive swerveSubsystem, Elevator elevatorSubsystem) {
        // If the claw is not holding coral, return an empty array
        if (!inputs.isCoralInClaw) {
            return new Pose3d[] {};
        }

        Translation2d coral2dPosition = swerveSubsystem
            .getActualPose()
            .getTranslation()
            .plus(
                getPose(elevatorSubsystem.getStage2Pose())
                    .getTranslation()
                    .toTranslation2d()
                    .plus(ClawConstants.RotationPositions.getClawToCoralX(inputs.turnPositionRad))
                    .rotateBy(swerveSubsystem.getActualPose().getRotation())
            );

        return new Pose3d[] {
            new Pose3d(
                coral2dPosition.getX(),
                coral2dPosition.getY(),
                getPose(elevatorSubsystem.getStage2Pose()).getZ() +
                ClawConstants.RotationPositions.getClawToCoralZ(inputs.turnPositionRad),
                new Rotation3d(
                    0,
                    inputs.turnPositionRad + ClawConstants.RotationPositions.CLAW_ANGLE_OFFSET.getRadians(),
                    swerveSubsystem.getActualPose().getRotation().getRadians()
                )
            ),
        };
    }
}
