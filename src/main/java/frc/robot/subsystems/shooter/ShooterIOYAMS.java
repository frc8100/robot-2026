package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;
import yams.mechanisms.velocity.FlyWheel;

public class ShooterIOYAMS implements ShooterIO {

    // Shoot motor
    protected final SparkMax shootMotor = new SparkMax(CANIdConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark shootMotorWrapped = new WrappedSpark(
        shootMotor,
        // TODO: not a good way to do this
        ShooterConstants.shootMotorConfig.withSubsystem(new Subsystem() {})
    );

    // Shooter Mechanism
    private FlyWheel shooter = new FlyWheel(ShooterConstants.shooterConfig.apply(shootMotorWrapped));

    private final MutAngularVelocity setpointVelocity = RadiansPerSecond.mutable(0.0);

    @Override
    public void setTargetExitVelocity(double velocityMetersPerSecond) {
        // TODO: Get velocity

        // shooter.setSpeed(velocityMetersPerSecond);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorConnected = shootMotorWrapped.updateData(inputs.motorData);

        inputs.setpointExitAngularVelocity.mut_replace(setpointVelocity);
    }
}
