package frc.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.Random;

/**
 * Generates noise based on velocity and a gaussian distribution.
 * The noise standard deviation increases with the absolute value of the velocity.
 */
public class VelocityNoiseGenerator {

    /**
     * Generates noise for Pose2d and Pose3d based on velocity.
     */
    public static class PoseVelocityNoiseGenerator {

        private final VelocityNoiseGenerator translationNoiseGenerator;
        private final VelocityNoiseGenerator rotationNoiseGenerator;

        /**
         * Constructs a PoseVelocityNoiseGenerator with specified parameters.
         * @param translationBaseStdDev - The base standard deviation for translation noise.
         * @param translationPerVelocityStdDev - The increase in standard deviation per unit of velocity for translation noise.
         * @param rotationBaseStdDev - The base standard deviation for rotation noise.
         * @param rotationPerVelocityStdDev - The increase in standard deviation per unit of velocity for rotation noise.
         */
        public PoseVelocityNoiseGenerator(
            Distance translationBaseStdDev,
            Distance translationPerVelocityStdDev,
            Angle rotationBaseStdDev,
            Angle rotationPerVelocityStdDev
        ) {
            this.translationNoiseGenerator = new VelocityNoiseGenerator(
                translationBaseStdDev.in(Meters),
                translationPerVelocityStdDev.in(Meters)
            );
            this.rotationNoiseGenerator = new VelocityNoiseGenerator(
                rotationBaseStdDev.in(Radians),
                rotationPerVelocityStdDev.in(Radians)
            );
        }

        /**
         * Applies noise to the given Pose2d based on the velocity.
         * @param pose - The original pose.
         * @param velocity - The current velocity.
         * @return The pose with added noise.
         */
        public Pose2d applyNoise(Pose2d pose, double velocity) {
            double noiseX = translationNoiseGenerator.generateNoise(velocity);
            double noiseY = translationNoiseGenerator.generateNoise(velocity);
            double noiseRotation = rotationNoiseGenerator.generateNoise(velocity);

            return new Pose2d(
                pose.getX() + noiseX,
                pose.getY() + noiseY,
                pose.getRotation().plus(new Rotation2d(noiseRotation))
            );
        }

        /**
         * Applies noise to the given pose based on the velocity.
         * @param pose - The original pose.
         * @param velocity - The current velocity.
         * @return The pose with added noise.
         */
        public Pose3d applyNoise(Pose3d pose, double velocity) {
            double noiseX = translationNoiseGenerator.generateNoise(velocity);
            double noiseY = translationNoiseGenerator.generateNoise(velocity);
            double noiseZ = 0.0;
            double noiseRotation = rotationNoiseGenerator.generateNoise(velocity);

            return new Pose3d(
                pose.getX() + noiseX,
                pose.getY() + noiseY,
                pose.getZ() + noiseZ,
                new Rotation3d(0, 0, pose.getRotation().getZ() + noiseRotation)
            );
        }
    }

    private final Random random = new Random();

    private final double baseStdDev;
    private final double perVelocityStdDev;

    /**
     * Constructs a VelocityNoiseGenerator with specified parameters.
     * @param baseStdDev - The base standard deviation of the noise.
     * @param perVelocityStdDev - The increase in standard deviation per unit of velocity.
     */
    public VelocityNoiseGenerator(double baseStdDev, double perVelocityStdDev) {
        this.baseStdDev = baseStdDev;
        this.perVelocityStdDev = perVelocityStdDev;
    }

    /**
     * Generates noise based on the given velocity.
     * A new random value is generated each time this method is called.
     * @param velocity - The current velocity.
     * @return The generated noise.
     */
    public double generateNoise(double velocity) {
        double stdDev = baseStdDev + perVelocityStdDev * Math.abs(velocity);
        return stdDev * random.nextGaussian();
    }
}
