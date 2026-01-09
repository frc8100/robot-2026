package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

/**
 * A test of YAMS
 */
public class TestArmSubsystem extends SubsystemBase {

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new ArmFeedforward(0, 0, 0))
        .withSimFeedforward(new ArmFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        // .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        .withGearing(new MechanismGearing(72))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    private final ArmConfig armCfg;
    private final Arm arm;

    private final TestArmSubsystemIO io;
    private final TestArmSubsystemIOInputsAutoLogged inputs = new TestArmSubsystemIOInputsAutoLogged();

    // TODO: add support for empty IO implementations
    public TestArmSubsystem() {
        io = new TestArmSubsystemIO(motorConfig);

        // Configure arm
        armCfg = new ArmConfig(io.getMotorController())
            // Soft limit is applied to the SmartMotorControllers PID
            .withSoftLimits(Degrees.of(-20), Degrees.of(10))
            // Hard limit is applied to the simulation.
            .withHardLimit(Degrees.of(-30), Degrees.of(40))
            // Starting position is where your arm starts
            .withStartingPosition(Degrees.of(-5))
            // Length and mass of your arm for sim.
            .withLength(Feet.of(3))
            .withMass(Pounds.of(1));

        arm = new Arm(armCfg);
    }

    /**
     * Set the angle of the arm.
     * @param angle Angle to go to.
     */
    public Command setAngle(Angle angle) {
        return arm.setAngle(angle);
    }

    /**
     * Move the arm up and down.
     * @param dutycycle [-1, 1] speed to set the arm too.
     */
    public Command set(double dutycycle) {
        return arm.set(dutycycle);
    }

    /**
     * Run sysId on the {@link Arm}
     */
    public Command sysId() {
        return arm.sysId(Volts.of(7), Volts.of(2).per(Seconds), Seconds.of(4));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TestArmSubsystem", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        arm.simIterate();
    }
}
