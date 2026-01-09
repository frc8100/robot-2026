package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

/**
 * The simulator implementation of the claw subsystem.
 * This is independent of the {@link ClawIOSim} class, which is the simulator implementation of the claw IO.
 */
public class ClawSim extends Claw {

    /**
     * Creates a new ClawSim. Automatically initializes the claw IO to a new instance of {@link ClawIOSim}.
     */
    public ClawSim() {
        super(new ClawIOSim());
    }

    /**
     * Releases the coral from the claw and creates a new {@link ReefscapeCoralOnFly} object in the simulator.
     * @param swerveSubsystem - The swerve subsystem.
     * @param elevatorSubsystem - The elevator subsystem.
     */
    private void releaseCoral(SwerveSim swerveSubsystem, Elevator elevatorSubsystem) {
        // Set the claw to not holding a piece of coral
        inputs.isCoralInClaw = false;

        // Create a new ReefscapeCoralOnFly object in the simulator
        ReefscapeCoralOnFly coralToAdd = new ReefscapeCoralOnFly(
            // The position of the robot
            swerveSubsystem.getActualPose().getTranslation(),
            // Where the claw is positioned
            super.getPose(elevatorSubsystem.getStage2Pose())
                .getTranslation()
                .toTranslation2d()
                .plus(ClawConstants.RotationPositions.getClawToCoralX(super.inputs.turnPositionRad)),
            // The speed of the robot
            swerveSubsystem.getChassisSpeeds(),
            // The rotation of the robot
            swerveSubsystem.getActualPose().getRotation(),
            // The vertical position of the claw
            Meters.of(
                super.getPose(elevatorSubsystem.getStage2Pose()).getZ() +
                ClawConstants.RotationPositions.getClawToCoralZ(super.inputs.turnPositionRad)
            ),
            // How fast to eject the coral
            ClawConstants.SIM_OUTTAKE_EJECT_SPEED,
            // The angle of the claw
            Radians.of(-(super.inputs.turnPositionRad + ClawConstants.RotationPositions.CLAW_ANGLE_OFFSET.getRadians()))
        );

        SimulatedArena.getInstance().addGamePieceProjectile(coralToAdd);
    }

    /**
     * Called in `Robot.simulationPeriodic()` to update the simulation.
     */
    public void simulationPeriodic(SwerveSim swerveSubsystem, Elevator elevatorSubsystem) {
        // If the claw is holding a piece of coral and the outtake is running, set the claw to not holding a piece of coral
        // and create a coral object in the simulator

        // TODO: Flip < or > depending on the direction of the outtake
        if (
            inputs.isCoralInClaw &&
            super.inputs.outtakeVelocityRadPerSec < ClawConstants.IntakeOuttakeDirection.OUTTAKE.getDirection() * 10
        ) {
            releaseCoral(swerveSubsystem, elevatorSubsystem);
        }

        // If the claw is intaking, assume that the claw is holding a piece of coral
        if (super.inputs.outtakeVelocityRadPerSec > ClawConstants.IntakeOuttakeDirection.BACK.getDirection() * 10) {
            inputs.isCoralInClaw = true;
        }
    }
}
