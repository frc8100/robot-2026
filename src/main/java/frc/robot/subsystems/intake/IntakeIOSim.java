package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.subsystems.vision.VisionConstants;
import frc.util.FuelSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class IntakeIOSim extends IntakeIOYAMS {

    private boolean isDeployed = false;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        super.deployMotorWrapped.setupSimulation();
        super.intakeMotorWrapped.setupSimulation();

        FuelSim.getInstance()
            .registerIntake(
                IntakeConstants.ROBOT_CENTER_TO_INTAKE_CENTER.getX(),
                IntakeConstants.ROBOT_CENTER_TO_INTAKE_CENTER.getX() + IntakeConstants.LENGTH.in(Meters),
                IntakeConstants.ROBOT_CENTER_TO_INTAKE_CENTER.getY() - IntakeConstants.WIDTH.in(Meters) / 2,
                IntakeConstants.ROBOT_CENTER_TO_INTAKE_CENTER.getY() + IntakeConstants.WIDTH.in(Meters) / 2,
                () -> isDeployed,
                this::onIntake
            );

        SimulatedBattery.addElectricalAppliances(super.intakeMotorWrapped::getStatorCurrent);
        SimulatedBattery.addElectricalAppliances(super.deployMotorWrapped::getStatorCurrent);
    }

    private void onIntake() {
        // TODO: do something
    }

    @Override
    public void deploy() {
        // TODO: rn this is instant deployment, make it take time later
        isDeployed = true;
    }

    @Override
    public void retract() {
        // intakeSimulation.stopIntake();
        isDeployed = false;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        super.updateInputs(inputs);

        inputs.measuredDeployState = isDeployed
            ? IntakeIO.MeasuredDeployState.DEPLOYED
            : IntakeIO.MeasuredDeployState.RETRACTED;
    }

    @Override
    public void simIterate() {
        super.deployMotorWrapped.simIterate();
        super.intakeMotorWrapped.simIterate();
    }
}
