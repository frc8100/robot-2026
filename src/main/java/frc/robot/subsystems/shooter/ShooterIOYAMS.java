package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.velocity.FlyWheel;

public class ShooterIOYAMS implements ShooterIO {

    // Shoot motor
    protected final SparkMax shootMotor = new SparkMax(CANIdConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark shootMotorWrapped = new WrappedSpark(
        shootMotor,
        // TODO: not a good way to do this
        ShooterConstants.shootMotorConfig.withSubsystem(new Subsystem() {})
    );

    // Indexer motor
    protected final SparkMax indexerMotor = new SparkMax(CANIdConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark indexerMotorWrapped = new WrappedSpark(
        indexerMotor,
        ShooterConstants.indexerMotorConfig
    );

    // Shooter Mechanism
    private FlyWheel shooter = new FlyWheel(ShooterConstants.shooterConfig.apply(shootMotorWrapped));

    @Override
    public void setTargetShootMotorVelocity(AngularVelocity velocity) {
        // test
        Logger.recordOutput("Shooter/TargetShootMotorVelocity", velocity);
        shootMotorWrapped.setVelocity(velocity);
    }

    @Override
    public void stopShooter() {
        shootMotorWrapped.setDutyCycle(0.0);
    }

    @Override
    public void runIndexer() {
        indexerMotorWrapped.setDutyCycle(ShooterConstants.INDEXER_OUTPUT);
    }

    @Override
    public void stopIndexer() {
        indexerMotorWrapped.setDutyCycle(0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shootMotorConnected = shootMotorWrapped.updateData(inputs.shootMotorData);
        inputs.indexerMotorConnected = indexerMotorWrapped.updateData(inputs.indexerMotorData);
    }
}
