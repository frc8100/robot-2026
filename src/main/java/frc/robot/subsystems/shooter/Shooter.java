package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem.
 */
public class Shooter extends SubsystemBase {

    public enum ShooterState {
        // /**
        //  * The intake is fully deployed.
        //  */
        // DEPLOYED,

        // /**
        //  * The intake is in transition between deployed and retracted.
        //  */
        // TRANSITION_DEPLOYING,

        // /**
        //  * The intake is in transition between retracted and deployed.
        //  */
        // TRANSITION_RETRACTING,

        // /**
        //  * The intake is fully retracted.
        //  */
        // RETRACTED,
    }

    public final StateMachine<ShooterState, Object> stateMachine = new StateMachine<ShooterState, Object>(
        ShooterState.class,
        "Shooter"
    );
    // .withDefaultState(new StateMachineState<>(ShooterState.RETRACTED, "Retracted"))
    // .withState(new StateMachineState<>(ShooterState.DEPLOYED, "Deployed"))
    // .withState(new StateMachineState<>(ShooterState.TRANSITION_DEPLOYING, "Transition Deploying"))
    // .withState(new StateMachineState<>(ShooterState.TRANSITION_RETRACTING, "Transition Retracting"));

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Swerve swerveSubsystem;

    public Shooter(ShooterIO io, Swerve swerveSubsystem) {
        this.io = io;
        this.swerveSubsystem = swerveSubsystem;
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
