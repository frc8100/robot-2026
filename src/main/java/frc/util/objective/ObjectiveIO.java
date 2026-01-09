package frc.util.objective;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectiveIO {
    @AutoLog
    public static class ObjectiveIOInputs {}

    public default void updateInputs(ObjectiveIOInputs inputs) {}
}
