package frc.robot.subsystems.swerve.setpoint;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.util.SwerveSetpointGenerator.SwerveSetpoint;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveSetpointIO {
    @AutoLog
    public static class SwerveSetpointIOInputs {

        public SwerveSetpoint[] unreadSetpoints = new SwerveSetpoint[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveSetpointIOInputs inputs) {}

    public default void setTargetChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {}
}
