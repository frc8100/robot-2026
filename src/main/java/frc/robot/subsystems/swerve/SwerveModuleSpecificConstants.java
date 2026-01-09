package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CANIdConnections;

/**
 * Contains module-specific constants for the swerve drive.
 * For CAN IDs, see {@link CANIdConnections}.
 * For general swerve constants, see {@link SwerveConstants}.
 */
public final class SwerveModuleSpecificConstants {

    private SwerveModuleSpecificConstants() {}

    /**
     * Swerve module constants to be used when creating swerve modules.
     * @param angleOffset - The angle offset of the module.
     */
    public static record RobotSwerveModuleConstants(Rotation2d angleOffset) {}

    /**
     * Gets the module constants for a module based on its index.
     * @param index - The index of the module (0-3). In FL, FR, BL, BR order.
     * @return The module constants for the module.
     * @throws IllegalArgumentException - If the index is not between 0 and 3
     */
    public static RobotSwerveModuleConstants getModuleConstantsFromIndex(int index) {
        return switch (index) {
            case 0 -> FRONT_LEFT_MODULE_CONSTANTS;
            case 1 -> FRONT_RIGHT_MODULE_CONSTANTS;
            case 2 -> BACK_LEFT_MODULE_CONSTANTS;
            case 3 -> BACK_RIGHT_MODULE_CONSTANTS;
            default -> throw new IllegalArgumentException("Invalid module index: " + index);
        };
    }

    // TODO: Redo these angle offsets; no need to include Math.PI adjustments if we set them correctly initially
    public static final RobotSwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = new RobotSwerveModuleConstants(
        new Rotation2d(2.922 + Math.PI - 0.626)
    );
    public static final RobotSwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = new RobotSwerveModuleConstants(
        new Rotation2d(2.635 - 0.222 - 0.845)
    );
    public static final RobotSwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = new RobotSwerveModuleConstants(
        new Rotation2d(1.267 + Math.PI - 0.004)
    );
    public static final RobotSwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = new RobotSwerveModuleConstants(
        new Rotation2d(-3.053 - 0.021)
    );
}
