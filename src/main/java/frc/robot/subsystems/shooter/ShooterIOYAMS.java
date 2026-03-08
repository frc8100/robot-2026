package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;
import yams.mechanisms.velocity.FlyWheel;

public class ShooterIOYAMS implements ShooterIO {

    // Shoot motor
    protected final SparkMax rightShootMotor = new SparkMax(
        CANIdConstants.RIGHT_SHOOTER_MOTOR_ID,
        MotorType.kBrushless
    );

    protected final SparkMax leftShootMotor = new SparkMax(CANIdConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark leftShootMotorWrapped = new WrappedSpark(
        leftShootMotor,
        // TODO: not a good way to do this
        DCMotor.getNEO(2),
        ShooterConstants.shootMotorConfig
            .withSubsystem(new Subsystem() {})
            .withFollowers(new Pair<>(rightShootMotor, false))
    );

    // Indexer motor
    protected final SparkMax indexerMotor = new SparkMax(CANIdConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark indexerMotorWrapped = new WrappedSpark(
        indexerMotor,
        ShooterConstants.indexerMotorConfig
    );

    // Shooter Mechanism
    protected final FlyWheel flywheel = new FlyWheel(ShooterConstants.shooterConfig.apply(leftShootMotorWrapped));

    protected boolean isUsingClosedLoopControlForShoot = true;
    protected boolean isUsingClosedLoopControlForIndexer = true;

    public ShooterIOYAMS() {
        enableClosedLoopControlShoot();
    }

    private void enableClosedLoopControlShoot() {
        if (isUsingClosedLoopControlForShoot) {
            return;
        }
        isUsingClosedLoopControlForShoot = true;

        leftShootMotorWrapped.overrideCurrentState(
            leftShootMotorWrapped.getMechanismVelocity(),
            // Assumes that the shooter is at rest when we enable closed loop control
            RadiansPerSecondPerSecond.zero()
        );
    }

    private void disableClosedLoopControlShoot() {
        isUsingClosedLoopControlForShoot = false;
        // shootMotorWrapped.stopClosedLoopController();
    }

    private void enableClosedLoopControlIndexer() {
        if (isUsingClosedLoopControlForIndexer) {
            return;
        }
        isUsingClosedLoopControlForIndexer = true;
    }

    private void disableClosedLoopControlIndexer() {
        isUsingClosedLoopControlForIndexer = false;
    }

    @Override
    public void setTargetShootMotorVelocity(AngularVelocity velocity) {
        enableClosedLoopControlShoot();
        leftShootMotorWrapped.setVelocity(velocity);
    }

    @Override
    public void stopShooter() {
        disableClosedLoopControlShoot();
        leftShootMotorWrapped.setDutyCycle(0.0);
    }

    @Override
    public void runShooterDutyCycle(Voltage dutyCycleOutput) {
        disableClosedLoopControlShoot();
        leftShootMotorWrapped.setVoltage(dutyCycleOutput);
    }

    @Override
    public void runIndexerDutyCycle(Voltage dutyCycleOutput) {
        disableClosedLoopControlIndexer();
        indexerMotorWrapped.setVoltage(dutyCycleOutput);
    }

    @Override
    public void stopIndexer() {
        disableClosedLoopControlIndexer();
        indexerMotorWrapped.setDutyCycle(0.0);
    }

    @Override
    public void setIndexerVelocitySetpoint(AngularVelocity velocity) {
        enableClosedLoopControlIndexer();
        indexerMotorWrapped.setVelocity(velocity);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shootMotorConnected = leftShootMotorWrapped.updateData(inputs.shootMotorData);
        inputs.indexerMotorConnected = indexerMotorWrapped.updateData(inputs.indexerMotorData);

        // Update the setpoints for shooter and indexer
        inputs.shootSetpoint.mut_replace(
            leftShootMotorWrapped.getMechanismSetpointVelocity().orElse(RadiansPerSecond.zero())
        );
        inputs.shootSetpointProfiled.mut_replace(leftShootMotorWrapped.currentState.position, RotationsPerSecond);
        inputs.shootSetpointAcceleration.mut_replace(
            leftShootMotorWrapped.currentState.velocity,
            RotationsPerSecondPerSecond
        );

        inputs.indexerSetpointProfiled.mut_replace(indexerMotorWrapped.currentState.position, RotationsPerSecond);
        inputs.indexerSetpointAcceleration.mut_replace(
            indexerMotorWrapped.currentState.velocity,
            RotationsPerSecondPerSecond
        );

        if (isUsingClosedLoopControlForShoot) {
            leftShootMotorWrapped.iterateCustomMotionProfile();
        }

        if (isUsingClosedLoopControlForIndexer) {
            indexerMotorWrapped.iterateCustomMotionProfile();
        }
    }
}
