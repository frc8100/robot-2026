package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
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
        DCMotor.getNEO(2),
        ShooterConstants.shootMotorConfig.withSubsystem(new Subsystem() {})
    );

    // Indexer motor
    protected final SparkMax indexerMotor = new SparkMax(CANIdConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark indexerMotorWrapped = new WrappedSpark(
        indexerMotor,
        ShooterConstants.indexerMotorConfig
    );

    // Shooter Mechanism
    protected final FlyWheel flywheel = new FlyWheel(ShooterConstants.shooterConfig.apply(shootMotorWrapped));

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
        // shootMotorWrapped.forceSetupClosedLoopController();
        // shootMotorWrapped.startClosedLoopController();

        shootMotorWrapped.overrideCurrentState(
            shootMotorWrapped.getMechanismVelocity(),
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
        shootMotorWrapped.setVelocity(velocity);
    }

    @Override
    public void stopShooter() {
        disableClosedLoopControlShoot();
        shootMotorWrapped.setDutyCycle(0.0);
    }

    @Override
    public void runShooterDutyCycle(Voltage dutyCycleOutput) {
        disableClosedLoopControlShoot();
        shootMotorWrapped.setVoltage(dutyCycleOutput);
    }

    @Override
    public void runIndexerDutyCycle(Voltage dutyCycleOutput) {
        disableClosedLoopControlIndexer();
        indexerMotorWrapped.setVoltage(dutyCycleOutput);
    }

    @Override
    public void setIndexerVelocitySetpoint(AngularVelocity velocity) {
        enableClosedLoopControlIndexer();
        indexerMotorWrapped.setVelocity(velocity);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shootMotorConnected = shootMotorWrapped.updateData(inputs.shootMotorData);
        inputs.indexerMotorConnected = indexerMotorWrapped.updateData(inputs.indexerMotorData);

        if (isUsingClosedLoopControlForShoot) {
            shootMotorWrapped.iterateCustomMotionProfile();
        }

        if (isUsingClosedLoopControlForIndexer) {
            indexerMotorWrapped.iterateCustomMotionProfile();
        }
    }
}
