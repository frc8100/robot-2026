// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class OdometryThread {

    /**
     * Lock for reading Phoenix signals.
     */
    private final Lock phoenixSignalsLock = new ReentrantLock();

    // Lists of registered signals and their queues
    private final List<SparkBase> sparks = new ArrayList<>();
    private final List<DoubleSupplier> sparkSignals = new ArrayList<>();

    private BaseStatusSignal[] allPhoenixAngleSignals = new BaseStatusSignal[0];

    private BaseStatusSignal[] phoenixAngleRotationsSignals = new BaseStatusSignal[0];
    private BaseStatusSignal[] phoenixAngleDegreesSignals = new BaseStatusSignal[0];

    // Corresponding queues for each signal
    private final List<Queue<Double>> sparkQueues = new ArrayList<>();
    private final List<Queue<Rotation2d>> phoenixAngleRotationsQueues = new ArrayList<>();
    private final List<Queue<Rotation2d>> phoenixAngleDegreesQueues = new ArrayList<>();

    private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);

    private static OdometryThread instance = null;
    private Notifier notifier = new Notifier(this::run);

    /**
     * @return The singleton instance of the odometry thread.
     */
    public static OdometryThread getInstance() {
        if (instance == null) {
            instance = new OdometryThread();
        }

        return instance;
    }

    /**
     * Private constructor for singleton pattern.
     */
    private OdometryThread() {
        notifier.setName("OdometryThread");
    }

    /**
     * Starts the odometry thread if any signals are registered.
     */
    public void start() {
        if (!sparkQueues.isEmpty() || !phoenixAngleRotationsQueues.isEmpty()) {
            notifier.startPeriodic(1.0 / SwerveConstants.ODOMETRY_FREQUENCY_HZ.in(Hertz));
        }
    }

    /**
     * Registers a Spark signal to be read from the thread.
     */
    public Queue<Double> registerSparkSignal(SparkBase spark, DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Swerve.odometryLock.lock();
        try {
            sparks.add(spark);
            sparkSignals.add(signal);
            sparkQueues.add(queue);

            sparkValuesCached = new double[sparkSignals.size()];
        } finally {
            Swerve.odometryLock.unlock();
        }
        return queue;
    }

    private void addPhoenixAngleSignal(StatusSignal<Angle> signal) {
        BaseStatusSignal[] newSignals = new BaseStatusSignal[allPhoenixAngleSignals.length + 1];
        System.arraycopy(allPhoenixAngleSignals, 0, newSignals, 0, allPhoenixAngleSignals.length);
        newSignals[allPhoenixAngleSignals.length] = signal;
        allPhoenixAngleSignals = newSignals;
    }

    /**
     * Registers a Phoenix angle signal to be read from the thread.
     * IMPORTANT: The StatusSignal must be configured to return values in rotations.
     */
    public Queue<Rotation2d> registerPhoenixAngleRotationsSignal(StatusSignal<Angle> signal) {
        Queue<Rotation2d> queue = new ArrayBlockingQueue<>(20);
        phoenixSignalsLock.lock();
        Swerve.odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixAngleRotationsSignals.length + 1];
            System.arraycopy(phoenixAngleRotationsSignals, 0, newSignals, 0, phoenixAngleRotationsSignals.length);
            newSignals[phoenixAngleRotationsSignals.length] = signal;
            phoenixAngleRotationsSignals = newSignals;

            addPhoenixAngleSignal(signal);

            phoenixAngleRotationsQueues.add(queue);
        } finally {
            phoenixSignalsLock.unlock();
            Swerve.odometryLock.unlock();
        }
        return queue;
    }

    /**
     * Registers a Phoenix angle signal to be read from the thread.
     * IMPORTANT: The StatusSignal must be configured to return values in degrees.
     */
    public Queue<Rotation2d> registerPhoenixAngleDegreesSignal(StatusSignal<Angle> signal) {
        Queue<Rotation2d> queue = new ArrayBlockingQueue<>(20);
        phoenixSignalsLock.lock();
        Swerve.odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixAngleDegreesSignals.length + 1];
            System.arraycopy(phoenixAngleDegreesSignals, 0, newSignals, 0, phoenixAngleDegreesSignals.length);
            newSignals[phoenixAngleDegreesSignals.length] = signal;
            phoenixAngleDegreesSignals = newSignals;

            addPhoenixAngleSignal(signal);

            phoenixAngleDegreesQueues.add(queue);
        } finally {
            phoenixSignalsLock.unlock();
            Swerve.odometryLock.unlock();
        }
        return queue;
    }

    /**
     * @return The queue that returns timestamp values for each sample.
     */
    public Queue<Double> getTimestampQueue() {
        return timestampQueue;
    }

    private double[] sparkValuesCached = new double[0];

    /**
     * Updates all registered signals and saves their values to their respective queues.
     */
    private void run() {
        // Wait for updates from all signals
        phoenixSignalsLock.lock();
        try {
            if (allPhoenixAngleSignals.length > 0) {
                // BaseStatusSignal.waitForAll(2.0 / SwerveConstants.ODOMETRY_FREQUENCY_HZ.in(Hertz), allPhoenixAngleSignals);
            } else {
                // "waitForAll" does not support blocking on multiple signals with a bus
                // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                // behavior is provided by the documentation.
                Thread.sleep((long) (1000.0 / SwerveConstants.ODOMETRY_FREQUENCY_HZ.in(Hertz)));
                if (allPhoenixAngleSignals.length > 0) BaseStatusSignal.refreshAll(allPhoenixAngleSignals);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            phoenixSignalsLock.unlock();
        }

        // Save new data to queues
        Swerve.odometryLock.lock();
        try {
            // Get sample timestamp
            double timestamp = RobotController.getFPGATime() / 1e6;

            // Read Spark values, mark invalid in case of error
            boolean isValid = true;
            for (int i = 0; i < sparkSignals.size(); i++) {
                sparkValuesCached[i] = sparkSignals.get(i).getAsDouble();
                if (sparks.get(i).getLastError() != REVLibError.kOk) {
                    isValid = false;
                }
            }

            // If valid, add values to queues
            if (isValid) {
                for (int i = 0; i < sparkSignals.size(); i++) {
                    sparkQueues.get(i).offer(sparkValuesCached[i]);
                }
                for (int i = 0; i < phoenixAngleRotationsSignals.length; i++) {
                    phoenixAngleRotationsQueues
                        .get(i)
                        .offer(Rotation2d.fromRotations(phoenixAngleRotationsSignals[i].getValueAsDouble()));
                }
                for (int i = 0; i < phoenixAngleDegreesSignals.length; i++) {
                    phoenixAngleDegreesQueues
                        .get(i)
                        .offer(Rotation2d.fromDegrees(phoenixAngleDegreesSignals[i].getValueAsDouble()));
                }

                timestampQueue.offer(timestamp);
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            Swerve.odometryLock.unlock();
        }
    }
}
