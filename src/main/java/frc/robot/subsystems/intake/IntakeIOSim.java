package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.util.FieldConstants;
import frc.util.FuelSim;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim extends IntakeIOYAMS {

    private boolean isDeployed = false;
    private int fuelInIntake = 0;

    /**
     * Stores the fuel positions as a transform relative to the robot.
     */
    private final List<Transform3d> fuelPositions = new ArrayList<>(IntakeConstants.MAX_CAPACITY);

    private final Swerve swerveSubsystem;

    public IntakeIOSim(Swerve swerveSubsystem) {
        super();
        this.swerveSubsystem = swerveSubsystem;

        // super.deployMotorWrapped.setupSimulation();
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
        // SimulatedBattery.addElectricalAppliances(super.deployMotorWrapped::getStatorCurrent);
    }

    private void onIntake() {
        fuelInIntake = Math.min(fuelInIntake + 1, IntakeConstants.MAX_CAPACITY);
    }

    public void removeFuelFromIntake() {
        fuelInIntake = Math.max(fuelInIntake - 1, 0);
    }

    /**
     * @return Whether the shooter is able to shoot based on if there is fuel in the intake.
     */
    public boolean isAbleToShoot() {
        return fuelInIntake > 0;
    }

    /**
     * Gets the position of the fuel in the intake based on its index in the intake.
     * @param fuelIndex - The index of the fuel in the intake, where 0 is the fuel closest to the front of the intake and higher indices are further back in the intake.
     * @return The position of the fuel relative to the robot.
     */
    private Transform3d getFuelPositionInIntake(int fuelIndex) {
        int layer = fuelIndex / (IntakeConstants.ROWS_OF_FUEL * IntakeConstants.COLUMNS_OF_FUEL);
        int indexInLayer = fuelIndex % (IntakeConstants.ROWS_OF_FUEL * IntakeConstants.COLUMNS_OF_FUEL);
        int row = indexInLayer / IntakeConstants.COLUMNS_OF_FUEL;
        int column = indexInLayer % IntakeConstants.COLUMNS_OF_FUEL;

        // TODO: move to constants
        final double START_X = SwerveConstants.SIDE_FRAME_LENGTH.in(Meters) / 2;
        final double START_Y = SwerveConstants.FRONT_FRAME_LENGTH.in(Meters) / 2;

        return new Transform3d(
            new Translation3d(
                START_X - (row * FieldConstants.fuelDiameter.in(Meters)),
                START_Y - (column * FieldConstants.fuelDiameter.in(Meters)),
                FieldConstants.fuelDiameter.in(Meters) * (layer + 1)
            ),
            Rotation3d.kZero
        );
    }

    /**
     * Updates the fuel positions based on the current number of fuel in the intake.
     */
    private void updateFuelPositions() {
        // If there is less fuel now than is stored, remove the most recently added fuel positions
        if (fuelPositions.size() > fuelInIntake) {
            for (int i = fuelPositions.size() - 1; i >= fuelInIntake; i--) {
                fuelPositions.remove(fuelPositions.size() - 1);
            }
        }

        // If there is more fuel now than is stored, add new fuel positions for the new fuel
        if (fuelPositions.size() < fuelInIntake) {
            for (int i = fuelPositions.size(); i < fuelInIntake; i++) {
                fuelPositions.add(getFuelPositionInIntake(fuelPositions.size()));
            }
        }
    }

    /**
     * @return The positions of the fuel in the intake relative to the field based on the robot pose.
     */
    public Translation3d[] getFuelPositions() {
        Pose3d robotPose = new Pose3d(swerveSubsystem.getActualPose());

        return fuelPositions
            .stream()
            .map(fuelPosition -> robotPose.transformBy(fuelPosition).getTranslation())
            .toArray(Translation3d[]::new);
    }

    @Override
    public void deploy() {
        // TODO: rn this is instant deployment, make it take time later
        isDeployed = true;
    }

    @Override
    public void retract() {
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
        // super.deployMotorWrapped.simIterate();
        super.intakeMotorWrapped.simIterate();

        updateFuelPositions();
        Logger.recordOutput("Intake/FuelInIntake", getFuelPositions());
    }
}
