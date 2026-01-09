package frc.robot;

/**
 * Contains all the CAN IDs used in the robot.
 * Also contains utilities for getting module CAN IDs.
 */
public class CANIdConstants {

    private CANIdConstants() {}

    public static final int PIGEON_ID = 17;

    /**
     * Swerve Module CAN IDs to be used when creating swerve modules.
     * @param driveMotorID - The CAN ID of the drive motor.
     * @param angleMotorID - The CAN ID of the angle motor.
     * @param canCoderID - The CAN ID of the CANCoder.
     */
    public static record SwerveModuleCanIDs(int driveMotorID, int angleMotorID, int canCoderID) {}

    // Swerve Module CAN IDs
    public static final SwerveModuleCanIDs FRONT_LEFT_MODULE_CAN_IDS = new SwerveModuleCanIDs(12, 3, 14);
    public static final SwerveModuleCanIDs FRONT_RIGHT_MODULE_CAN_IDS = new SwerveModuleCanIDs(5, 2, 13);
    public static final SwerveModuleCanIDs BACK_LEFT_MODULE_CAN_IDS = new SwerveModuleCanIDs(4, 10, 15);
    public static final SwerveModuleCanIDs BACK_RIGHT_MODULE_CAN_IDS = new SwerveModuleCanIDs(1, 8, 16);

    /**
     * Gets the CAN IDs for a module based on its index.
     * @param index - The index of the module (0-3). In FL, FR, BL, BR order.
     * @return The CAN IDs for the module.
     * @throws IllegalArgumentException - If the index is not between 0 and 3.
     */
    public static SwerveModuleCanIDs getModuleCANIdsFromIndex(int index) {
        return switch (index) {
            case 0 -> FRONT_LEFT_MODULE_CAN_IDS;
            case 1 -> FRONT_RIGHT_MODULE_CAN_IDS;
            case 2 -> BACK_LEFT_MODULE_CAN_IDS;
            case 3 -> BACK_RIGHT_MODULE_CAN_IDS;
            default -> throw new IllegalArgumentException("Invalid module index: " + index);
        };
    }

    /**
     * List of all CAN IDs in the order the CAN bus is wired, starting from the PDP and ending at the RoboRIO.
     * This is used to detect connection disruptions.
     */
    // TODO: set this
    public static final int[] canIdConnectionsInOrder = new int[] {
        PIGEON_ID,
        FRONT_LEFT_MODULE_CAN_IDS.driveMotorID,
        FRONT_LEFT_MODULE_CAN_IDS.angleMotorID,
        FRONT_LEFT_MODULE_CAN_IDS.canCoderID,
        FRONT_RIGHT_MODULE_CAN_IDS.driveMotorID,
        FRONT_RIGHT_MODULE_CAN_IDS.angleMotorID,
        FRONT_RIGHT_MODULE_CAN_IDS.canCoderID,
        BACK_LEFT_MODULE_CAN_IDS.driveMotorID,
        BACK_LEFT_MODULE_CAN_IDS.angleMotorID,
        BACK_LEFT_MODULE_CAN_IDS.canCoderID,
        BACK_RIGHT_MODULE_CAN_IDS.driveMotorID,
        BACK_RIGHT_MODULE_CAN_IDS.angleMotorID,
        BACK_RIGHT_MODULE_CAN_IDS.canCoderID,
    };
}
