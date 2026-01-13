// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.CANIdConstants;
import frc.robot.subsystems.swerve.OdometryThread;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.util.AntiTipping;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 pigeon = new Pigeon2(CANIdConstants.PIGEON_ID);

    // Status signals
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private final StatusSignal<Angle> pitch = pigeon.getPitch();
    private final StatusSignal<Angle> roll = pigeon.getRoll();

    // Odometry queues
    private final Queue<Rotation2d> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    // Anti-tipping
    private final AntiTipping antiTipping = new AntiTipping(
        SwerveConstants.ANTI_TIPPING_KP,
        SwerveConstants.TIPPING_THRESHOLD.in(Degrees),
        SwerveConstants.MAX_ANTI_TIP_VELOCITY.in(MetersPerSecond)
    );

    public GyroIOPigeon2() {
        // Configure the pigeon
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);

        yaw.setUpdateFrequency(SwerveConstants.ODOMETRY_FREQUENCY_HZ);
        yawVelocity.setUpdateFrequency(SwerveConstants.STATUS_SIGNAL_FREQUENCY_HZ);

        // Only update pitch and roll if configured to do so
        pitch.setUpdateFrequency(
            SwerveConstants.IS_GYRO_RECORD_PITCH_ROLL_TIPPING_STATE ? SwerveConstants.STATUS_SIGNAL_FREQUENCY_HZ : 0
        );
        roll.setUpdateFrequency(
            SwerveConstants.IS_GYRO_RECORD_PITCH_ROLL_TIPPING_STATE ? SwerveConstants.STATUS_SIGNAL_FREQUENCY_HZ : 0
        );

        pigeon.optimizeBusUtilization();

        // Register odometry signals
        yawTimestampQueue = OdometryThread.getInstance().getTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerPhoenixAngleDegreesSignal(yaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // Refresh all signals and set connected status
        if (SwerveConstants.IS_GYRO_RECORD_PITCH_ROLL_TIPPING_STATE) {
            // If anti-tipping is enabled, refresh pitch and roll as well
            inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity, pitch, roll).equals(StatusCode.OK);
        } else {
            inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        }

        // Update from status signals
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocity.mut_replace(yawVelocity.getValueAsDouble(), DegreesPerSecond);

        if (SwerveConstants.IS_GYRO_RECORD_PITCH_ROLL_TIPPING_STATE) {
            // Update pitch and roll only if configured to do so
            inputs.pitch.mut_replace(pitch.getValueAsDouble(), Degrees);
            inputs.roll.mut_replace(roll.getValueAsDouble(), Degrees);

            // Update anti-tipping
            antiTipping.calculate(inputs.pitch, inputs.roll);
            inputs.isTipping = antiTipping.isTipping();
            inputs.velocityAntiTipping = antiTipping.getVelocityAntiTipping();
        }

        // Update odometry caches
        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.toArray(new Rotation2d[0]);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void setYaw(double deg) {
        // Invert the gyro if necessary
        if (SwerveConstants.IS_GYRO_INVERTED) {
            deg = -deg;
        }

        // Zero the gyro and update the odometry
        pigeon.setYaw(deg);
    }
}
