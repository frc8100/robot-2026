package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.module.ModuleIO;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Simulator implementation of the swerve drive.
 */
public class SwerveSim extends Swerve {

    /**
     * The simulation object for the swerve drive.
     * Contains the simulated drivetrain pose and other simulation-specific methods.
     */
    private final SwerveDriveSimulation driveSimulation;

    /**
     * Creates a new SwerveSim object.
     * @param gyroIO The gyro IO object.
     * @param moduleIOs The array of module IO objects.
     * @param driveSimulation The simulation object for the swerve drive.
     */
    public SwerveSim(GyroIO gyroIO, ModuleIO[] moduleIOs, SwerveDriveSimulation driveSimulation) {
        super(gyroIO, moduleIOs);
        this.driveSimulation = driveSimulation;

        // Simulation specific SysId
        super.sysId = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.75).per(Seconds), Volts.of(10), Seconds.of(15), state ->
                Logger.recordOutput("Swerve/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(voltage -> runCharacterization(voltage.in(Volts)), null, this)
        );
    }

    @Override
    public boolean isSimulation() {
        return true;
    }

    @Override
    @AutoLogOutput(key = "Odometry/Field")
    public Pose2d getActualPose() {
        return driveSimulation.getSimulatedDriveTrainPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        super.setPose(pose);

        driveSimulation.setSimulationWorldPose(pose);
    }
    // @Override
    // public void periodic() {
    //     super.periodic();
    //     // Send simulation data to dashboard for testing
    //     // field.setRobotPose(driveSimulation.getSimulatedDriveTrainPose());
    //     // field.getObject("odometry").setPose(getPose());
    // }
}
