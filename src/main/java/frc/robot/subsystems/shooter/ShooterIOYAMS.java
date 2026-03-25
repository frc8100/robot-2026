package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.CANIdConstants;
import frc.util.SubsystemIOUtil;
import frc.util.TunableValue;
import frc.util.WrappedSpark;
import yams.mechanisms.velocity.FlyWheel;

public class ShooterIOYAMS implements ShooterIO {

    // Shoot motor

    protected final SparkMax leaderShootMotor = new SparkMax(
        CANIdConstants.LEFT_SHOOTER_MOTOR_ID,
        MotorType.kBrushless
    );
    protected final WrappedSpark leaderShootMotorWrapped = new WrappedSpark(
        leaderShootMotor,
        // TODO: not a good way to do this
        DCMotor.getNEO(2),
        ShooterConstants.shootMotorConfig.withSubsystem(new Subsystem() {})
        // .withFollowers(new Pair<>(rightShootMotor, true))
    );

    protected final SparkMax followerShootMotor = new SparkMax(
        CANIdConstants.RIGHT_SHOOTER_MOTOR_ID,
        MotorType.kBrushless
    );
    protected final RelativeEncoder followerEncoder = followerShootMotor.getEncoder();

    // Indexer motor
    protected final SparkMax indexerMotor = new SparkMax(CANIdConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark indexerMotorWrapped = new WrappedSpark(
        indexerMotor,
        ShooterConstants.indexerMotorConfig
    );

    // Shooter Mechanism
    protected final FlyWheel flywheel = new FlyWheel(ShooterConstants.shooterConfig.apply(leaderShootMotorWrapped));

    protected boolean isUsingClosedLoopControlForShoot = true;
    protected boolean isUsingClosedLoopControlForIndexer = true;

    /**
     * Beam breaker to detect whether fuel is at the top of the indexer. When channel 0 is false, there is fuel at the top of the indexer.
     * Channel 1 is the opposite of channel 0, and is used to detect whether the beam breaker is connected. If channel 0 and channel 1 are the same, then the beam breaker is not connected.
     */
    private final DigitalInput fuelInShooterBeamBreaker = new DigitalInput(0);
    private final DigitalInput fuelInShooterBeamBreakerOpposite = new DigitalInput(1);

    // public final TunableValue.SparkPIDTunable deployTunablePID = TunableValue.SparkPIDTunable.fromWrapped(
    //     "Intake/Deploy",
    //     deployMotorWrapped
    // );
    // public final TunableValue.SparkFeedforwardTunable deployTunableFF = new TunableValue.SparkFeedforwardTunable(
    //     "Intake/Deploy",
    //     deployMotorWrapped
    // );

    public final TunableValue.SparkPIDTunable shooterTunablePID = TunableValue.SparkPIDTunable.fromWrapped(
        "Shooter/Shooter",
        leaderShootMotorWrapped
    );
    public final TunableValue.SparkFeedforwardTunable deployTunableFF = new TunableValue.SparkFeedforwardTunable(
        "Shooter/Shooter",
        leaderShootMotorWrapped
    );

    public final TunableValue.SparkPIDTunable indexerTunablePID = TunableValue.SparkPIDTunable.fromWrapped(
        "Shooter/Indexer",
        indexerMotorWrapped
    );
    public final TunableValue.SparkFeedforwardTunable indexerTunableFF = new TunableValue.SparkFeedforwardTunable(
        "Shooter/Indexer",
        indexerMotorWrapped
    );

    public ShooterIOYAMS() {
        // Configure follower
        SparkBaseConfig followerConfig = new SparkMaxConfig().apply(leaderShootMotorWrapped.getSparkConfig());
        followerConfig.follow(leaderShootMotor, true);
        followerShootMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        enableClosedLoopControlShoot();
    }

    private void enableClosedLoopControlShoot() {
        if (isUsingClosedLoopControlForShoot) {
            return;
        }
        isUsingClosedLoopControlForShoot = true;

        leaderShootMotorWrapped.overrideCurrentState(
            leaderShootMotorWrapped.getMechanismVelocity(),
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
        leaderShootMotorWrapped.setCustomSetpointVelocity(velocity);
    }

    @Override
    public void stopShooter() {
        disableClosedLoopControlShoot();
        leaderShootMotorWrapped.setDutyCycle(0.0);
    }

    @Override
    public void runShooterDutyCycle(Voltage dutyCycleOutput) {
        disableClosedLoopControlShoot();
        leaderShootMotorWrapped.setVoltage(dutyCycleOutput);
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
        indexerMotorWrapped.setCustomSetpointVelocity(velocity);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leaderShootMotorConnected = leaderShootMotorWrapped.updateData(inputs.leaderShootMotorData);
        inputs.indexerMotorConnected = indexerMotorWrapped.updateData(inputs.indexerMotorData);

        // inputs.isFuelDetectedAtTopOfIndexer = !fuelInShooterBeamBreaker.get();
        // XOR the two channels to determine if the beam breaker is connected
        // inputs.beamBreakerConnected = fuelInShooterBeamBreaker.get() ^ fuelInShooterBeamBreakerOpposite.get();

        inputs.followerShootMotorConnected = SubsystemIOUtil.updateDataFromSpark(
            inputs.followerShootMotorData,
            followerShootMotor,
            followerEncoder
        );

        // Update the setpoints for shooter and indexer
        inputs.shootSetpoint.mut_replace(
            leaderShootMotorWrapped.getMechanismSetpointVelocity().orElse(RadiansPerSecond.zero())
        );
        inputs.shootSetpointProfiled.mut_replace(leaderShootMotorWrapped.currentState.position, RotationsPerSecond);
        inputs.shootSetpointAcceleration.mut_replace(
            leaderShootMotorWrapped.currentState.velocity,
            RotationsPerSecondPerSecond
        );

        inputs.indexerSetpointProfiled.mut_replace(indexerMotorWrapped.currentState.position, RotationsPerSecond);
        inputs.indexerSetpointAcceleration.mut_replace(
            indexerMotorWrapped.currentState.velocity,
            RotationsPerSecondPerSecond
        );

        if (isUsingClosedLoopControlForShoot) {
            leaderShootMotorWrapped.iterateCustomMotionProfile();
        }

        if (isUsingClosedLoopControlForIndexer) {
            indexerMotorWrapped.iterateCustomMotionProfile();
        }
    }
}
