package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

/**
 * The simulator implementation for the claw.
 */
public class ClawIOSim implements ClawIO {

    /**
     * The simulated angle motor. Controls the angle of the claw.
     */
    private final MapleMotorSim angleMotorSim = new MapleMotorSim(ClawConstants.SIM_ANGLE_MOTOR_CONFIG);

    /**
     * The simulated angle motor controller.
     */
    private final SimulatedMotorController.GenericMotorController angleMotorController;

    /**
     * The PID angle controller for the claw.
     */
    private final PIDController angleController = new PIDController(
        ClawConstants.SIM_ANGLE_KP,
        ClawConstants.SIM_ANGLE_KI,
        ClawConstants.SIM_ANGLE_KD
    );

    /**
     * Whether the angle controller is using PID or not.
     * If false, the angle motor will not be updated by the PID controller.
     * The PID controller will still be updated.
     */
    private boolean isAngleUsingPID = true;

    /**
     * The simulated outtake motor. Controls the outtake of the claw.
     */
    private final MapleMotorSim outtakeMotorSim = new MapleMotorSim(ClawConstants.SIM_OUTTAKE_MOTOR_CONFIG);

    /**
     * The simulated outtake motor controller.
     */
    private final SimulatedMotorController.GenericMotorController outtakeMotorController;

    /**
     * The PID outtake controller for the claw.
     */
    private final PIDController outtakeController = new PIDController(
        ClawConstants.SIM_OUTTAKE_KP,
        ClawConstants.SIM_OUTTAKE_KI,
        ClawConstants.SIM_OUTTAKE_KD
    );

    /**
     * Whether the outtake controller is using PID or not.
     * If false, the outtake motor will not be updated by the PID controller.
     * The PID controller will still be updated.
     */
    private boolean isOuttakeUsingPID = true;

    /**
     * Creates a new ClawIOSim.
     */
    public ClawIOSim() {
        // Assign the motor controllers
        angleMotorController = angleMotorSim
            .useSimpleDCMotorController()
            .withCurrentLimit(ClawConstants.ANGLE_MOTOR_CURRENT_LIMIT);

        outtakeMotorController = outtakeMotorSim
            .useSimpleDCMotorController()
            .withCurrentLimit(ClawConstants.OUTTAKE_MOTOR_CURRENT_LIMIT);

        // Reset the controllers
        angleController.reset();
        outtakeController.reset();
    }

    @Override
    public void stop() {
        // Stop the angle motor
        isAngleUsingPID = false;
        isOuttakeUsingPID = false;

        angleMotorController.requestVoltage(Volts.of(0));
        outtakeMotorController.requestVoltage(Volts.of(0));
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        isAngleUsingPID = true;

        // Set the setpoint of the angle controller
        angleController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void increaseTurnPosition(double valueToIncreaseBy) {
        isAngleUsingPID = true;

        // Increase the setpoint of the angle controller
        angleController.setSetpoint(angleController.getSetpoint() + valueToIncreaseBy);
    }

    @Override
    public void runOuttake(double motorInput) {
        isOuttakeUsingPID = true;

        // Set the setpoint of the outtake controller
        outtakeController.setSetpoint(motorInput * ClawConstants.SIM_OUTTAKE_TARGET_VELOCITY.in(RadiansPerSecond));
    }

    @Override
    public void periodic() {
        // Set the output of the motors based on the PID controller
        double angleMotorOutput = angleController.calculate(angleMotorSim.getAngularPosition().in(Radians));

        if (isAngleUsingPID) {
            Logger.recordOutput("Simulation/Claw/AnglePIDOutput", MathUtil.clamp(angleMotorOutput, -12, 12));
            angleMotorController.requestVoltage(Volts.of(MathUtil.clamp(angleMotorOutput, -12, 12)));
        }

        double outtakeMotorOutput = outtakeController.calculate(outtakeMotorSim.getVelocity().in(RadiansPerSecond));

        if (isOuttakeUsingPID) {
            Logger.recordOutput("Simulation/Claw/OuttakePIDOutput", MathUtil.clamp(outtakeMotorOutput, -12, 12));
            outtakeMotorController.requestVoltage(Volts.of(MathUtil.clamp(outtakeMotorOutput, -12, 12)));
        }

        // Update the simulation
        angleMotorSim.update(Seconds.of(Constants.LOOP_PERIOD_SECONDS));
        outtakeMotorSim.update(Seconds.of(Constants.LOOP_PERIOD_SECONDS));
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        // Set angle inputs
        inputs.turnConnected = true;
        inputs.turnPositionRad = angleMotorSim.getAngularPosition().in(Radians);
        inputs.turnVelocityRadPerSec = angleMotorSim.getVelocity().in(RadiansPerSecond);
        inputs.turnAppliedVolts = angleMotorSim.getAppliedVoltage().in(Volts);
        inputs.turnSupplyCurrentAmps = angleMotorSim.getSupplyCurrent().in(Amps);
        inputs.turnTorqueCurrentAmps = angleMotorSim.getStatorCurrent().in(Amps);
        inputs.turnSetpointRad = angleController.getSetpoint();

        // Set outtake inputs
        inputs.outtakeConnected = true;
        inputs.outtakePositionRad = outtakeMotorSim.getAngularPosition().in(Radians);
        inputs.outtakeVelocityRadPerSec = outtakeMotorSim.getVelocity().in(RadiansPerSecond);
        inputs.outtakeAppliedVolts = outtakeMotorSim.getAppliedVoltage().in(Volts);
        inputs.outtakeSupplyCurrentAmps = outtakeMotorSim.getSupplyCurrent().in(Amps);
        inputs.outtakeTorqueCurrentAmps = outtakeMotorSim.getStatorCurrent().in(Amps);
        inputs.outtakeSetpointVelocityRadPerSec = outtakeController.getSetpoint();

        // Simulate coral detection based on outtake velocity
        inputs.isCoralInClaw = outtakeMotorSim.getVelocity().in(RadiansPerSecond) > 20;
    }
}
