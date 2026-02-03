package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem.
 */
public class Shooter extends SubsystemBase {

    public enum ShooterState {
        /**
         * Not shooting.
         */
        IDLE,

        /**
         * The shooter is spinning to the target velocity and the indexer is running to move fuel to the shooter.
         */
        SHOOTING,
    }

    public final StateMachine<ShooterState, Object> stateMachine = new StateMachine<ShooterState, Object>(
        ShooterState.class,
        "Shooter"
    )
        .withDefaultState(new StateMachineState<>(ShooterState.IDLE, "Idle"))
        .withState(new StateMachineState<>(ShooterState.SHOOTING, "Shooting"));

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Swerve swerveSubsystem;

    public Shooter(ShooterIO io, Swerve swerveSubsystem) {
        this.io = io;
        this.swerveSubsystem = swerveSubsystem;

        // State machine bindings
        stateMachine.whileState(ShooterState.IDLE, () -> {
            setTargetExitVelocity(0.0);
        });

        stateMachine.whileState(ShooterState.SHOOTING, this::handleShootState);
    }

    private void handleShootState() {
        // TODO: set based on auto aim calculation
        setTargetExitVelocity(
            swerveSubsystem.autoAim.latestCalculationResult.getTargetFuelExitVelocity().in(MetersPerSecond)
        );
    }

    public void testShoot() {
        io.testShoot();
    }

    public void setTargetExitVelocity(double velocityMetersPerSecond) {
        io.setTargetExitVelocity(velocityMetersPerSecond);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        setTargetExitVelocity(
            swerveSubsystem.autoAim.latestCalculationResult.getTargetFuelExitVelocity().in(MetersPerSecond)
        );
    }
}
