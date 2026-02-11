package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Declares the key values for different values.
 * The behavior for these keys are handed in {@link ButtonBindings}.
 */
public class ControlConstants {

    private ControlConstants() {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Shooter controls
    public static final XboxController.Button toggleShoot = XboxController.Button.kA;

    // Intake controls
    public static final XboxController.Button toggleIntakeDeploy = XboxController.Button.kLeftBumper;
    public static final XboxController.Button runIntakeButton = XboxController.Button.kB;

    // Drive controls
    public static final XboxController.Button toggleAutoAimToHub = XboxController.Button.kX;
    public static final ButtonBindings.Controller.POVButtonDirection toggleAutoDriveIntake =
        ButtonBindings.Controller.POVButtonDirection.DOWN;

    // Operator controls

    // Simulation controls (on operator controller)
    public static final XboxController.Button spawnAllFuelButton = XboxController.Button.kBack;
    public static final XboxController.Button clearFuelButton = XboxController.Button.kStart;

    public static final Drive mainDriveControls = new Drive(ButtonBindings.driverController);
    public static final Drive joystickDriveControls = new JoystickDrive(ButtonBindings.driverController);

    /**
     * Whether or not to use the joystick drive.
     */
    public static final boolean USE_JOYSTICK_DRIVE = false;

    /** The drive controls */
    public static class Drive {

        private GenericHID driverController;

        /**
         * Creates a new drive controls object.
         * @param driverController The driver controller
         */
        protected Drive(GenericHID driverController) {
            this.driverController = driverController;
        }

        /** Whether to invert the drive controls. Default is `true`. */
        protected boolean invertDriveControls = true;

        // Driver Controls
        // By default, the left stick controls robot movement (translation - y, strafe - x)
        // and the right stick controls the rotation (x)
        public int translationAxis = XboxController.Axis.kLeftY.value;
        public int strafeAxis = XboxController.Axis.kLeftX.value;
        public int rotationAxis = XboxController.Axis.kRightX.value;

        // Driver Buttons
        /**
         * When pressed, zeroes the yaw offset.
         * Press when robot is facing towards the drive station to align the robot's forward direction with the field.
         */
        public final XboxController.Button zeroYawOffsetButton = XboxController.Button.kY;

        /**
         * When held, slows the robot down to {@link #slowMultiplier}
         */
        public final XboxController.Button slowButton = XboxController.Button.kRightBumper;

        public final double slowMultiplier = 0.5;

        /**
         * @return The translation (x)
         */
        public double getTranslationValue() {
            return invertDriveControls
                ? -driverController.getRawAxis(translationAxis)
                : driverController.getRawAxis(translationAxis);
        }

        /**
         * @return The strafe (y)
         */
        public double getStrafeValue() {
            return invertDriveControls
                ? -driverController.getRawAxis(strafeAxis)
                : driverController.getRawAxis(strafeAxis);
        }

        /**
         * @return The rotation
         */
        public double getRotationValue() {
            return invertDriveControls
                ? -driverController.getRawAxis(rotationAxis)
                : driverController.getRawAxis(rotationAxis);
        }

        /**
         * @return Whether the controls are robot centric. Default is `false`.
         */
        public boolean isRobotCentric() {
            return false;
        }

        /**
         * @return The speed multiplier.
         */
        public double getSpeedMultiplier() {
            return driverController.getRawButton(slowButton.value) ? slowMultiplier : 1;
        }
    }

    /**
     * Drive controls using a joystick
     */
    protected static class JoystickDrive extends Drive {

        protected JoystickDrive(GenericHID driverController) {
            super(driverController);
            // Override axis values
            translationAxis = Joystick.AxisType.kY.value;
            strafeAxis = Joystick.AxisType.kX.value;
            rotationAxis = Joystick.AxisType.kZ.value;
        }
    }
}
