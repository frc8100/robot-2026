package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * IO implementation for the gyro simulation.
 */
public class GyroIOSim implements GyroIO {

    /**
     * Use the maplesim gyro simulation. This contains the gyro readings and angular velocity.
     */
    private final GyroSimulation gyroSimulation;

    /**
     * Creates a new GyroIOSim given the gyro simulation to use.
     */
    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocity.mut_replace(gyroSimulation.getMeasuredAngularVelocity());

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }

    @Override
    public void setYaw(double deg) {
        gyroSimulation.setRotation(Rotation2d.fromDegrees(deg));
    }
}
