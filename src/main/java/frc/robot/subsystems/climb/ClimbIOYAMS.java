package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;

public class ClimbIOYAMS implements ClimbIO {

    protected final SparkMax rightClimbMotor = new SparkMax(CANIdConstants.RIGHT_CLIMB_MOTOR_ID, MotorType.kBrushless);

    protected final SparkMax leftClimbMotor = new SparkMax(CANIdConstants.LEFT_CLIMB_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark leftClimbMotorWrapped = new WrappedSpark(
        leftClimbMotor,
        ClimbConstants.climbMotorConfig.apply(rightClimbMotor)
    );
}
