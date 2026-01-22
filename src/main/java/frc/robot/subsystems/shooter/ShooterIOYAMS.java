package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;
import yams.mechanisms.velocity.FlyWheel;

public class ShooterIOYAMS implements ShooterIO {

    // Shoot motor
    protected final SparkMax shootMotor = new SparkMax(CANIdConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark shootMotorWrapped = new WrappedSpark(shootMotor, ShooterConstants.shootMotorConfig);

    // Shooter Mechanism
    private FlyWheel shooter = new FlyWheel(ShooterConstants.shooterConfig.apply(shootMotorWrapped));

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorConnected = shootMotorWrapped.updateData(inputs.motorData);
    }
}
