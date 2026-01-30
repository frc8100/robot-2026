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

// yall ts not tuff at allllll -Layla

package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CANIdConstants;
import frc.robot.subsystems.swerve.OdometryThread;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleSpecificConstants;
import frc.robot.subsystems.swerve.SwerveModuleSpecificConstants.RobotSwerveModuleConstants;
import frc.util.SparkUtil;
import frc.util.SubsystemIOUtil;
import frc.util.TunableValue;
import java.util.Queue;

/**
 * Module IO implementation for Spark Max drive and angle motors with a CANcoder for absolute angle.
 */
public class ModuleIOSpark implements ModuleIO {

    public final int moduleNumber;

    /**
     * The dashboard key for this module. Used for logging.
     * Example: "Swerve/Module1"
     */
    private final String dashboardKey;

    /**
     * The angle offset. Used to zero the module to a specific angle.
     */
    private final Rotation2d angleOffset;

    /**
     * The angle motor. This motor is used to control the angle of the module.
     * Includes the integrated encoder {@link #relativeAngleEncoder} and an absolute CANcoder {@link #angleCANcoder}.
     */
    private final SparkMax angleMotor;

    private final CANcoder angleCANcoder;
    private final RelativeEncoder relativeAngleEncoder;
    private final SparkClosedLoopController angleClosedLoopController;

    /**
     * The drive motor. This motor is used to control the speed of the module.
     * Includes an integrated encoder {@link #relativeDriveEncoder}.
     */
    private final SparkMax driveMotor;

    private final RelativeEncoder relativeDriveEncoder;
    private final SparkClosedLoopController driveClosedLoopController;

    // Status signal queues from CANcoder
    private final StatusSignal<Angle> turnAbsolutePosition;

    // Queue inputs from odometry thread
    private final Queue<Double> drivePositionQueue;
    private final Queue<Rotation2d> turnPositionQueue;

    // Inputs (cached)
    private double driveFFVolts = 0.0;
    private double angleFFVolts = 0.0;

    // Tunable values
    // private final TunableValue.SparkPIDTunable drivePidTunable;
    // private final TunableValue.SparkPIDTunable anglePidTunable;

    /**
     * Creates a new Swerve Module. Automatically gets CAN IDs and module constants based on module number.
     * @param moduleNumber - The module number.
     */
    public ModuleIOSpark(int moduleNumber) {
        this(
            moduleNumber,
            CANIdConstants.getModuleCANIdsFromIndex(moduleNumber),
            SwerveModuleSpecificConstants.getModuleConstantsFromIndex(moduleNumber)
        );
    }

    /**
     * Creates a new Swerve Module.
     * @param moduleNumber - The module number.
     * @param canIDs - The CAN IDs for the module.
     * @param moduleConstants - The module-specific constants.
     */
    public ModuleIOSpark(
        int moduleNumber,
        CANIdConstants.SwerveModuleCanIDs canIDs,
        RobotSwerveModuleConstants moduleConstants
    ) {
        // Set the module number and angle offset
        this.moduleNumber = moduleNumber;
        this.dashboardKey = "Swerve/Module" + moduleNumber;
        this.angleOffset = Rotation2d.fromRotations(moduleConstants.angleOffset());

        // Create and configure the angle motor
        angleMotor = new SparkMax(canIDs.angleMotorID(), MotorType.kBrushless);
        relativeAngleEncoder = angleMotor.getEncoder();
        angleClosedLoopController = angleMotor.getClosedLoopController();
        SparkUtil.configure(angleMotor, SwerveConstants.getAngleMotorConfig());

        // Create and configure the drive motor
        driveMotor = new SparkMax(canIDs.driveMotorID(), MotorType.kBrushless);
        relativeDriveEncoder = driveMotor.getEncoder();
        driveClosedLoopController = driveMotor.getClosedLoopController();
        SparkUtil.configure(driveMotor, SwerveConstants.getDriveMotorConfig());

        // Create and configure the CANCoder
        angleCANcoder = new CANcoder(canIDs.canCoderID());
        angleCANcoder.getConfigurator().refresh(new CANcoderConfiguration());

        // Apply the angle offset to the CANCoder configuration
        var cancoderConfig = SwerveConstants.getCANcoderConfig();
        cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.angleOffset();
        angleCANcoder.getConfigurator().apply(cancoderConfig);

        turnAbsolutePosition = angleCANcoder.getAbsolutePosition();
        turnAbsolutePosition.setUpdateFrequency(SwerveConstants.ODOMETRY_FREQUENCY_HZ);
        angleCANcoder.optimizeBusUtilization();

        // Reset the module to absolute position
        syncMotorEncoderToAbsoluteEncoder();

        // Create odometry queues
        drivePositionQueue = OdometryThread.getInstance()
            .registerSparkSignal(driveMotor, relativeDriveEncoder::getPosition);
        turnPositionQueue = OdometryThread.getInstance().registerPhoenixAngleRotationsSignal(turnAbsolutePosition);

        // Create tunable values
        // drivePidTunable = new TunableValue.SparkPIDTunable(
        //     this.dashboardKey,
        //     driveMotor,
        //     SwerveConstants.getDriveMotorConfig(),
        //     SwerveConstants.driveKP,
        //     SwerveConstants.driveKD
        // );
        // anglePidTunable = new TunableValue.SparkPIDTunable(
        //     this.dashboardKey,
        //     angleMotor,
        //     SwerveConstants.getAngleMotorConfig(),
        //     SwerveConstants.angleKP,
        //     SwerveConstants.angleKD
        // );

        TunableValue.addRefreshConfigConsumer(this::onRefresh);
    }

    /**
     * Refreshes the motor configurations from TunableValues.
     */
    private void onRefresh() {
        // Print raw turn angle
        System.out.println(
            "Module " + moduleNumber + " Raw CANCoder Angle (radians): " + getAngle().plus(angleOffset).getRadians()
        );

        var newConfigAngle = SwerveConstants.getAngleMotorConfig();
        newConfigAngle.closedLoop.p(SwerveConstants.angleKPTunable.get()).d(SwerveConstants.angleKDTunable.get());
        angleMotor.configure(newConfigAngle, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        var newConfigDrive = SwerveConstants.getDriveMotorConfig();
        newConfigDrive.closedLoop.p(SwerveConstants.driveKPTunable.get()).d(SwerveConstants.driveKDTunable.get());
        driveMotor.configure(newConfigDrive, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * @return The angle of the module, gotten from the CANCoder.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    }

    @Override
    public void syncMotorEncoderToAbsoluteEncoder() {
        double absolutePosition = getAngle().getRadians();
        relativeAngleEncoder.setPosition(absolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        inputs.driveMotorConnected = SubsystemIOUtil.updateDataFromSpark(
            inputs.driveMotorData,
            driveMotor,
            relativeDriveEncoder,
            driveClosedLoopController
        );
        inputs.driveFFVolts = driveFFVolts;

        // Check CANCoder connection
        // CANCoder signals should be already refreshed from the odometry thread
        inputs.canCoderConnected = turnAbsolutePosition.getStatus().isOK();

        // Update turn inputs
        turnAbsolutePosition.refresh();
        inputs.turnAbsolutePosition = getAngle();
        inputs.turnMotorConnected = SubsystemIOUtil.updateDataFromSpark(
            inputs.turnMotorData,
            angleMotor,
            relativeAngleEncoder,
            angleClosedLoopController
        );

        // Update odometry inputs
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.toArray(new Rotation2d[0]);
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        angleMotor.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(SwerveModuleState desiredState, double driveFeedforwardVoltage) {
        double velocityRadiansPerSecond = desiredState.speedMetersPerSecond / SwerveConstants.WHEEL_RADIUS.in(Meters);

        this.driveFFVolts = driveFeedforwardVoltage;

        driveClosedLoopController.setSetpoint(
            velocityRadiansPerSecond,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            driveFeedforwardVoltage,
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setTurnPosition(SwerveModuleState desiredState, double angleFeedforwardVoltage) {
        // Stop the motor if the speed is less than 1%. Prevents Jittering
        if (
            Math.abs(getAngle().minus(desiredState.angle).getDegrees()) < 1 &&
            Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_SPEED.in(MetersPerSecond) * 0.0125)
        ) {
            angleMotor.stopMotor();
            return;
        }

        this.angleFFVolts = angleFeedforwardVoltage;

        // Set the angle using the PID controller
        Rotation2d angle = desiredState.angle;
        double radReference = angle.getRadians();
        angleClosedLoopController.setSetpoint(
            radReference,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            angleFeedforwardVoltage,
            ArbFFUnits.kVoltage
        );
    }
}
