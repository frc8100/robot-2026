package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ShooterCharacterization;
import frc.robot.commands.SwerveSysidRoutines;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOYAMS;
import frc.robot.subsystems.questnav.QuestNavIO;
import frc.robot.subsystems.questnav.QuestNavIOReal;
import frc.robot.subsystems.questnav.QuestNavIOSim;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionState;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import frc.robot.subsystems.vision.VisionSim;
import frc.robot.subsystems.vision.VisionSim.NeuralDetectorSimPipeline;
import frc.util.CustomSimulationArena;
import frc.util.EmptySimulationArena;
import frc.util.FuelSim;
import frc.util.TunableValue;
import frc.util.objective.ObjectiveIO;
import frc.util.objective.ObjectiveIODashboard;
import frc.util.statemachine.StateMachine.StateWithPayload;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
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
    private final Intake intakeSubsystem;
    private final Shooter shooterSubsystem;
    private final Climb climbSubsystem;

    private final RobotActions robotActions;

    private ObjectiveIO objectiveIO = new ObjectiveIO() {};

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
                // swerveSubsystem = new Swerve(
                //     new GyroIOPigeon2(),
                //     new ModuleIO[] {
                //         new ModuleIOSpark(0),
                //         new ModuleIOSpark(1),
                //         new ModuleIOSpark(2),
                //         new ModuleIOSpark(3),
                //     }
                // );
                swerveSubsystem = new Swerve(
                    new GyroIO() {},
                    new ModuleIO[] { new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {} }
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

                // TODO: replace with real IO implementation when ready
                // intakeSubsystem = new Intake(new IntakeIOYAMS());
                intakeSubsystem = new Intake(new IntakeIO() {});
                shooterSubsystem = new Shooter(new ShooterIO() {}, swerveSubsystem);
                climbSubsystem = new Climb(new ClimbIO() {});

                objectiveIO = new ObjectiveIODashboard();
                break;
            default:
            case SIM:
                // Override the arena
                if (Constants.disableSimArena) {
                    // Use an empty arena for SysId to reduce obstacles
                    SimulatedArena.overrideInstance(new EmptySimulationArena());
                } else {
                    SimulatedArena.overrideInstance(new CustomSimulationArena(false));
                }

                SimulatedArena.getInstance().clearGamePieces();

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

                // Fuel simulation
                FuelSim.getInstance()
                    .registerRobot(
                        SwerveConstants.FRONT_FRAME_LENGTH.in(Meters),
                        SwerveConstants.SIDE_FRAME_LENGTH.in(Meters),
                        SwerveConstants.BUMPER_HEIGHT.in(Meters),
                        driveSimulation::getSimulatedDriveTrainPose,
                        driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative
                    );
                FuelSim.getInstance().start();

                // Create a simulated vision subsystem
                NeuralDetectorSimPipeline[] simPipelines = VisionSim.getDetectorPipelines(
                    // SimulatedArena.getInstance()::getGamePiecesPosesByType
                    (String className) ->
                        FuelSim.getInstance()
                            .fuels.stream()
                            .map(fuel -> new Pose3d(fuel.getPosition(), Rotation3d.kZero))
                            .toList()
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

                IntakeIOSim intakeIO = new IntakeIOSim(swerveSubsystem);

                intakeSubsystem = new Intake(intakeIO);
                shooterSubsystem = new Shooter(
                    new ShooterIOSim(swerveSubsystem, intakeIO::removeFuelFromIntake, intakeIO::isAbleToShoot),
                    swerveSubsystem
                );
                climbSubsystem = new Climb(new ClimbIOSim());

                objectiveIO = new ObjectiveIODashboard();

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
                intakeSubsystem = new Intake(new IntakeIO() {});
                shooterSubsystem = new Shooter(new ShooterIO() {}, swerveSubsystem);
                climbSubsystem = new Climb(new ClimbIO() {});
                objectiveIO = new ObjectiveIO() {};
                break;
        }
        // Set up auto routines
        robotActions = new RobotActions(
            swerveSubsystem,
            visionSubsystem,
            intakeSubsystem,
            shooterSubsystem,
            climbSubsystem,
            objectiveIO
        );

        // Register PathPlanner named commands
        NamedCommands.registerCommands(swerveSubsystem.stateMachine.getNamedCommands());
        NamedCommands.registerCommand(
            "SwerveAimToHub",
            Commands.runOnce(() -> {
                swerveSubsystem.stop();
                swerveSubsystem.stateMachine.scheduleStateChange(
                    new StateWithPayload<>(Swerve.SwerveState.AUTO_AIM, RobotActions.POINT_TO_HUB_PAYLOAD)
                );
            })
        );
        NamedCommands.registerCommands(visionSubsystem.stateMachine.getNamedCommands());
        NamedCommands.registerCommands(intakeSubsystem.stateMachine.getNamedCommands());
        NamedCommands.registerCommand("RunIntakeRoller", intakeSubsystem.runRollerCommand());
        NamedCommands.registerCommand("StopIntakeRoller", intakeSubsystem.stopRollerCommand());
        NamedCommands.registerCommands(shooterSubsystem.stateMachine.getNamedCommands());
        NamedCommands.registerCommands(climbSubsystem.stateMachine.getNamedCommands());

        Swerve.configurePathPlannerAutoBuilder(swerveSubsystem, questNavSubsystem);

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        if (Constants.enableSysId) {
            setupSysIdRoutines();
        }

        autoChooser.addDefaultOption("Actually move forward", robotActions.actuallyMoveForward());
        autoChooser.addOption("Move forward specific distance", robotActions.moveForwardSpecificDistance());

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

        // Drive SysId routines
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

        // Currently only dynamic to test constraints
        autoChooser.addOption(
            "Drive Angle SysId (Dynamic Forward)",
            swerveSubsystem.angleSysIdDynamic(SysIdRoutine.Direction.kForward)
        );

        autoChooser.addOption(
            "Drive Angle SysId (All 4)",
            new SequentialCommandGroup(
                swerveSubsystem.angleSysIdQuasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                swerveSubsystem.angleSysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1),
                swerveSubsystem.angleSysIdDynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                swerveSubsystem.angleSysIdDynamic(SysIdRoutine.Direction.kReverse)
            )
        );

        // Shooter SysId routines
        autoChooser.addOption("Shooter SysId", shooterSubsystem.shooterSysidCommand());
        autoChooser.addOption("Shooter Shooting Characterization", new ShooterCharacterization(shooterSubsystem));

        autoChooser.addOption("Intake SysId", intakeSubsystem.sysid());
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
        // if (!(swerveSubsystem instanceof SwerveSim && clawSubsystem instanceof ClawSim)) {
        //     throw new IllegalStateException("Subsystems are not simulated");
        // }
    }

    /**
     * Run in `Robot.periodic()`.
     */
    public void periodic() {
        if (!Constants.shouldLogAdditionalData()) {
            return;
        }

        Pose3d robotPose3d = new Pose3d(swerveSubsystem.getActualPose());
        Pose3d pivotLocation = robotPose3d.transformBy(ClimbConstants.CLIMB_CENTER_OF_ROTATION);

        Logger.recordOutput(
            "Climb/RobotPose",
            robotPose3d.rotateAround(pivotLocation.getTranslation(), climbSubsystem.getRobotTransform())
        );
    }

    /**
     * Sets the vision subsystem state to {@link Vision.VisionState#BEFORE_MATCH}.
     * See {@link Vision.VisionState} for more details.
     */
    // TODO: find other way to trigger it
    public void setVisionStateToBeforeMatch() {
        visionSubsystem.stateMachine.scheduleStateChange(Vision.VisionState.BEFORE_MATCH);
    }

    /**
     * Runs initialization code for autonomous mode.
     */
    public void autonomousInit() {
        // Set vision to during match
        visionSubsystem.stateMachine.scheduleStateChange(Vision.VisionState.DURING_MATCH);

        // Reset objective
        objectiveIO.resetForAuto();
    }

    /**
     * Runs initialization code for teleop mode.
     */
    public void teleopInit() {
        // Should already be during match, but just in case
        if (!visionSubsystem.stateMachine.is(VisionState.DURING_MATCH)) {
            visionSubsystem.stateMachine.scheduleStateChange(Vision.VisionState.DURING_MATCH);
        }

        // Reset objective
        objectiveIO.resetForTeleop();
    }

    /**
     * Runs initialization code for disabled mode.
     */
    public void disabledInit() {
        // Reset objective
        objectiveIO.resetForDisabled();
    }
}
