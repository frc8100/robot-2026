package frc.util.statemachine;

import edu.wpi.first.wpilibj.util.Color;

/**
 * A possible state that a {@link StateMachine} can be in. Contains additional metadata about the state.
 * @param <TEnumType> - The enum type representing the states of the state machine.
 */
public class StateMachineState<TEnumType extends Enum<TEnumType>> {

    @FunctionalInterface
    public interface StateChangeCondition<TEnumType extends Enum<TEnumType>> {
        /**
         * A functional interface representing a condition to determine if a state change can occur.
         * @param previousState - The previous state before the change.
         */
        public boolean canChange(TEnumType previousState);
    }

    /**
     * The enum type representing the state. This is used for equality checks.
     */
    public final TEnumType enumType;

    /**
     * The name of the state, used for logging and debugging.
     * Convention is to use PascalCase for state names.
     */
    public final String name;

    /**
     * A supplier that determines if the state can be changed to this state.
     * By default, it always returns true.
     */
    public StateChangeCondition<TEnumType> canChangeCondition = previousState -> true;

    /**
     * The color associated with this state for visualization.
     */
    public Color color = Color.kBlack;

    /**
     * Constructs a StateMachineState with the specified enum type and name.
     * @param enumType - The enum type representing the state.
     * @param name - The name of the state. Should be in PascalCase.
     */
    public StateMachineState(TEnumType enumType, String name) {
        this.enumType = enumType;
        this.name = name;
    }

    /**
     * Sets the condition that determines if the state can be changed to this state.
     */
    public StateMachineState<TEnumType> withCanChangeCondition(StateChangeCondition<TEnumType> condition) {
        this.canChangeCondition = condition;
        return this;
    }

    /**
     * Sets the color associated with this state.
     */
    public StateMachineState<TEnumType> withColor(Color color) {
        this.color = color;
        return this;
    }

    @Override
    public String toString() {
        return name;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        StateMachineState<?> other = (StateMachineState<?>) obj;
        return enumType.equals(other.enumType);
    }

    @Override
    public int hashCode() {
        return enumType.hashCode();
    }
}
