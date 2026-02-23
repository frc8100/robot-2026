package frc.robot.subsystems.climb;

public class ClimbIOSim extends ClimbIOYAMS {

    @Override
    public void simIterate() {
        super.leftClimbMotorWrapped.simIterate();
    }
}
