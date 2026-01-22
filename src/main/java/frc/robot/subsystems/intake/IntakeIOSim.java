package frc.robot.subsystems.intake;

import frc.robot.subsystems.vision.VisionConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim extends IntakeIOYAMS {

    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        // Create the intake simulation with respect to the intake on your real robot
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            VisionConstants.GamePieceObservationType.FUEL.className,
            driveTrain,
            IntakeConstants.WIDTH,
            IntakeConstants.LENGTH,
            IntakeConstants.ORIENTATION,
            IntakeConstants.MAX_CAPACITY
        );
    }
}
