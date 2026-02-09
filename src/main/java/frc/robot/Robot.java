// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CANIdConnections;
import frc.robot.subsystems.swerve.OpponentRobotSim;
import frc.util.FuelSim;
import frc.util.SparkUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    public Robot() {
        // Set the current mode automatically if it is not replay
        if (Constants.currentMode != Constants.Mode.REPLAY) {
            Constants.currentMode = isReal() ? Constants.Mode.REAL : Constants.Mode.SIM;
        }

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
        Logger.recordMetadata("SimulationState", Constants.currentMode.toString());

        DriverStation.silenceJoystickConnectionWarning(Constants.silenceJoystickUnpluggedWarning);

        // Enable/disable vendor loggers
        com.ctre.phoenix6.SignalLogger.enableAutoLogging(Constants.ENABLE_CTRE_SIGNAL_LOGGER);
        if (!Constants.ENABLE_REV_SIGNAL_LOGGER) {
            com.revrobotics.util.StatusLogger.disableAutoLogging();
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(
                    new WPILOGWriter(Constants.TUNING_MODE ? Constants.realLogDirectory : "/U/logs")
                );
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                // If enabled, log to a WPILOG file
                if (Constants.storeSimLogs) {
                    Logger.addDataReceiver(new WPILOGWriter(Constants.simLogDirectory));
                }

                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                // Replaying a log, set up replay source
                // false to run as fast as possible
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Initialize URCL
        if (Constants.ENABLE_URCL) {
            Logger.registerURCL(URCL.startExternal());
        }

        // Start AdvantageKit logger
        Logger.start();

        // Instantiate RobotContainer
        robotContainer = new RobotContainer();

        SparkUtil.warmupErrorLogging();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);

        CANIdConnections.periodic();

        // RobotContainer periodic
        robotContainer.periodic();

        SparkUtil.periodic();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // If in simulation, reset the simulated arena
        if (Constants.currentMode == Constants.Mode.SIM) {
            SimulatedArena.getInstance().resetFieldForAuto();
        }

        robotContainer.autonomousInit();

        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the autonomous command
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when autonomous ends. */
    @Override
    public void autonomousExit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        robotContainer.teleopInit();

        // TODO: bind this to a keypress for testing
        if (Constants.currentMode == Constants.Mode.SIM) {
            SimulatedArena.getInstance().clearGamePieces();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        // Skip if not in simulation mode
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }

        SimulatedArena.getInstance().simulationPeriodic();
        RoboRioSim.setVInCurrent(SimulatedBattery.getTotalCurrentDrawn().in(Amps));
        robotContainer.simulationPeriodic();
        FuelSim.getInstance().updateSim();

        // Log output of opponent robots
        Logger.recordOutput("Odometry/OpponentRobotPoses", OpponentRobotSim.getOpponentRobotPoses());

        // Log game pieces
        // Logger.recordOutput("Simulation/Field/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        FuelSim.getInstance().logFuels();
    }
}
