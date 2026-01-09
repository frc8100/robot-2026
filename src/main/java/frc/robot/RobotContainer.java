package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveSysidRoutines;
import frc.robot.subsystems.CANIdConnections;
import frc.robot.subsystems.questnav.QuestNavIO;
import frc.robot.subsystems.questnav.QuestNavIOReal;
import frc.robot.subsystems.questnav.QuestNavIOSim;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawIO;
import frc.robot.subsystems.superstructure.claw.ClawIOSpark;
import frc.robot.subsystems.superstructure.claw.ClawSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSpark;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleSpecificConstants;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.VisionSim.NeuralDetectorSimPipeline;
import frc.util.EmptySimulationArena;
import frc.util.TunableValue;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Vision visionSubsystem;
    private final QuestNavSubsystem questNavSubsystem;
    private final Swerve swerveSubsystem;
    private final Claw clawSubsystem;
    private final Elevator elevatorSubsystem;

    private final RobotActions robotActions;

    /**
     * The simulation of the robot's drive. Set to null if not in simulation mode.
     */
    private SwerveDriveSimulation driveSimulation = null;

    /**
     * Chooses the auto command.
     */
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveSubsystem = new Swerve(
                    new GyroIOPigeon2(),
                    new ModuleIO[] {
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3),
                    }
                );

                questNavSubsystem = new QuestNavSubsystem(swerveSubsystem::addVisionMeasurement, new QuestNavIOReal());
                visionSubsystem = new Vision(
                    swerveSubsystem,
                    questNavSubsystem,
                    new VisionIOLimelight(
                        VisionConstants.CAMERA_0_NAME,
                        VisionConstants.TRANSFORM_TO_CAMERA_0,
                        swerveSubsystem
                    )
                );

                clawSubsystem = new Claw(new ClawIOSpark());
                elevatorSubsystem = new Elevator(new ElevatorIOSpark());
                break;
            default:
            case SIM:
                // Override the arena
                if (Constants.disableSimArena) {
                    // Use an empty arena for SysId to reduce obstacles
                    SimulatedArena.overrideInstance(new EmptySimulationArena());
                } else {
                    // Use the reefscape arena for normal sim
                    SimulatedArena.overrideInstance(new Arena2025Reefscape());
                }

                SimulatedArena.getInstance().placeGamePiecesOnField();

                // Create a simulated drive
                driveSimulation = new SwerveDriveSimulation(
                    SwerveConstants.mapleSimConfig,
                    SwerveConstants.initialPose
                );
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                SwerveModuleSimulation[] moduleSims = driveSimulation.getModules();
                swerveSubsystem = new SwerveSim(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIO[] {
                        new ModuleIOSim(moduleSims[0]),
                        new ModuleIOSim(moduleSims[1]),
                        new ModuleIOSim(moduleSims[2]),
                        new ModuleIOSim(moduleSims[3]),
                    },
                    driveSimulation
                );

                // Create a simulated vision subsystem
                NeuralDetectorSimPipeline[] simPipelines = VisionSim.getDetectorPipelines(
                    SimulatedArena.getInstance()::getGamePiecesPosesByType
                );

                questNavSubsystem = new QuestNavSubsystem(
                    swerveSubsystem::addVisionMeasurement,
                    new QuestNavIOSim(swerveSubsystem)
                );

                visionSubsystem = new VisionSim(
                    swerveSubsystem,
                    simPipelines,
                    questNavSubsystem,
                    new VisionIOPhotonSim(
                        VisionConstants.CAMERA_0_NAME,
                        VisionConstants.TRANSFORM_TO_CAMERA_0,
                        VisionConstants.CAMERA_0_PROPERTIES,
                        swerveSubsystem,
                        simPipelines
                    )
                );

                // Create a simulated claw
                clawSubsystem = new ClawSim();

                elevatorSubsystem = new Elevator(new ElevatorIOSim());

                // TODO: Add behavior chooser
                // Create an opponent robot simulation
                // OpponentRobotSim opponentRobotSim1 = new OpponentRobotSim(
                //     new Pose2d(10, 2, new Rotation2d()),
                //     OpponentRobotBehavior.FOLLOW_PATH
                // );

                // Create another joystick drive for the opponent robot
                // Controls.Drive opponentRobotDriveControls = new Controls.JoystickDrive(new Joystick(1));

                // Set the default command for the opponent robot
                // opponentRobotSim1.setDefaultCommand(
                //         new TeleopSwerve(opponentRobotSim1, opponentRobotDriveControls, false));
                // opponentRobotSim1.setDefaultCommand(
                //     opponentRobotSim1.opponentRobotPathfindToPoseSupplier(swerveSubsystem::getActualPose)
                // );

                // OpponentRobotSim opponentRobotSim2 = new OpponentRobotSim(
                //     new Pose2d(10, 2, new Rotation2d()),
                //     OpponentRobotBehavior.FOLLOW_PATH
                // );
                // opponentRobotSim2.setDefaultCommand(
                //     opponentRobotSim2.opponentRobotPathfindToPoseSupplier(swerveSubsystem::getActualPose)
                // );

                // TODO: refactor
                // opponentRobotDriveControls
                //         .getJoystickButtonOf(opponentRobotDriveControls.zeroGyroButton)
                //         .onTrue(new InstantCommand(() -> opponentRobotSim1.zeroGyro()));
                break;
            case REPLAY:
                // Replay mode, instantiate dummy IO implementations
                swerveSubsystem = new Swerve(
                    new GyroIO() {},
                    new ModuleIO[] { new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {} }
                );
                questNavSubsystem = new QuestNavSubsystem(swerveSubsystem::addVisionMeasurement, new QuestNavIO() {});
                visionSubsystem = new Vision(swerveSubsystem, questNavSubsystem, new VisionIO() {});
                clawSubsystem = new Claw(new ClawIO() {});
                elevatorSubsystem = new Elevator(new ElevatorIO() {});
                break;
        }

        SwerveDrive.configurePathPlannerAutoBuilder(swerveSubsystem, questNavSubsystem);

        // Set up auto routines
        robotActions = new RobotActions(swerveSubsystem, elevatorSubsystem, clawSubsystem, visionSubsystem);

        // Set up teleop swerve command
        swerveSubsystem.setDefaultCommand(swerveSubsystem.stateMachine.getRunnableCommand(swerveSubsystem));

        // Register named commands
        NamedCommands.registerCommand(
            "ResetSuperstructure",
            robotActions.setUpSuperstructure(SuperstructureConstants.Level.INITIAL_POSITION)
        );

        NamedCommands.registerCommand(
            "SetupSuperstructureL1Auto",
            robotActions.setUpSuperstructure(SuperstructureConstants.Level.L1_AUTO)
        );
        NamedCommands.registerCommand(
            "SetupSuperstructureL2",
            robotActions.setUpSuperstructure(SuperstructureConstants.Level.L2)
        );
        NamedCommands.registerCommand(
            "SetupSuperstructureL3",
            robotActions.setUpSuperstructure(SuperstructureConstants.Level.L3)
        );
        NamedCommands.registerCommand(
            "SetupSuperstructureL4",
            robotActions.setUpSuperstructure(SuperstructureConstants.Level.L4)
        );

        NamedCommands.registerCommand("ScoreL4", robotActions.doClawMovementsForL4());
        NamedCommands.registerCommand("ScoreCoral", clawSubsystem.runOuttakeUntilCoralIsNotInClaw());

        NamedCommands.registerCommand("IntakeCoral", clawSubsystem.runIntakeUntilCoralIsInClaw());

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        if (Constants.enableSysId) {
            setupSysIdRoutines();
        }

        autoChooser.addOption("Coral and Go To All Reefs Test", robotActions.getCoralAndGoToAllReefsTest());

        autoChooser.addDefaultOption("Actually move forward", robotActions.actuallyMoveForward());
        autoChooser.addOption("Push another robot forward", robotActions.pushAnotherRobotForward());

        autoChooser.addOption("runIntakeUntilCoralIsInClaw", clawSubsystem.runIntakeUntilCoralIsInClaw());
        autoChooser.addOption("runOuttakeUntilCoralIsNotInClaw", clawSubsystem.runOuttakeUntilCoralIsNotInClaw());

        // Command to refresh the config
        SmartDashboard.putData("RefreshTunableConfig", TunableValue.getRefreshConfigCommand());

        // Configure the button bindings
        ButtonBindings buttonBindings = new ButtonBindings(robotActions);
        buttonBindings.configureButtonBindings();
        buttonBindings.assignDefaultCommands();

        if (Constants.currentMode == Constants.Mode.SIM) {
            // Sim button bindings
            buttonBindings.configureSimulationBindings();
        }

        // Warmup pathfinding
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    /**
     * Adds SysId routines to the auto chooser.
     */
    private void setupSysIdRoutines() {
        // Simple characterization routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            SwerveSysidRoutines.wheelRadiusCharacterization(swerveSubsystem)
        );
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            SwerveSysidRoutines.feedforwardCharacterization(swerveSubsystem)
        );
        autoChooser.addOption(
            "Drive Wheel Slip Current Characterization",
            SwerveSysidRoutines.wheelSlipCurrentCharacterization(swerveSubsystem)
        );
        autoChooser.addOption(
            "Max Acceleration and Velocity Test",
            swerveSubsystem.runMaxAccelerationMaxVelocityTest()
        );

        autoChooser.addOption(
            "QuestNav Transform Measure",
            questNavSubsystem.getMeasureTransformCommand(swerveSubsystem)
        );

        // Actual SysId routines
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        autoChooser.addOption(
            "Drive SysId (All 4)",
            new SequentialCommandGroup(
                swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1),
                swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Run in `Robot.simulationPeriodic()` to update the subsystem-specific simulation.
     * @throws IllegalStateException if the subsystems are not simulated
     */
    public void simulationPeriodic() {
        // Check all the subsystems are simulated
        if (!(swerveSubsystem instanceof SwerveSim && clawSubsystem instanceof ClawSim)) {
            throw new IllegalStateException("Subsystems are not simulated");
        }

        // Update the simulation
        ((ClawSim) clawSubsystem).simulationPeriodic((SwerveSim) swerveSubsystem, elevatorSubsystem);
    }

    /**
     * Run in `Robot.periodic()`.
     */
    public void periodic() {
        // Update telemetry for claw position
        Logger.recordOutput("ComponentPositions/Claw", clawSubsystem.getPose(elevatorSubsystem.getStage2Pose()));
        Logger.recordOutput(
            "ComponentPositions/CoralInClaw",
            clawSubsystem.getCoralInClawPosition(swerveSubsystem, elevatorSubsystem)
        );
    }

    /**
     * Sets the vision subsystem state to {@link Vision.VisionState#BEFORE_MATCH}.
     * See {@link Vision.VisionState} for more details.
     */
    public void setVisionStateToBeforeMatch() {
        visionSubsystem.stateMachine.scheduleStateChange(Vision.VisionState.BEFORE_MATCH);
    }

    /**
     * Sets the vision subsystem state to {@link Vision.VisionState#DURING_MATCH}.
     * See {@link Vision.VisionState} for more details.
     */
    public void setVisionStateToMatchStart() {
        visionSubsystem.stateMachine.scheduleStateChange(Vision.VisionState.DURING_MATCH);
    }
}
