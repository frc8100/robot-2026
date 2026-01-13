package frc.util.statemachine;

import java.util.List;
import java.util.Optional;

/**
 * A cycle of states. Mainly used for controller inputs that toggle/cycle through a set of states.
 */
public class StateCycle<TStateEnum extends Enum<TStateEnum>, TPayload> {

    /**
     * The behavior of the state cycle when cycling through states.
     */
    public enum StateCycleBehavior {
        /**
         * Rely on the current state of the state machine to determine the next state in the cycle.
         */
        RELY_ON_CURRENT_STATE,

        /**
         * Rely on the index of the state in the cycle to determine the next state.
         */
        RELY_ON_INDEX;

        /**
         * The default behavior of the state cycle.
         * This is {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}.
         */
        public static final StateCycleBehavior DEFAULT = RELY_ON_CURRENT_STATE;
    }

    private final StateMachine<TStateEnum, TPayload> stateMachine;
    private final List<StateMachine.StateWithPayload<TStateEnum, TPayload>> states;
    private final StateCycleBehavior behavior;

    private int currentIndex = 0;

    /**
     * Creates a new StateCycle with the specified states and behavior.
     * @param stateMachine - The state machine to cycle through.
     * @param states - The states to cycle through.
     * @param behavior - The behavior of the state cycle.
     */
    public StateCycle(
        StateMachine<TStateEnum, TPayload> stateMachine,
        List<StateMachine.StateWithPayload<TStateEnum, TPayload>> states,
        StateCycleBehavior behavior
    ) {
        this.stateMachine = stateMachine;
        this.states = states;
        this.behavior = behavior;
    }

    /**
     * Creates a new StateCycle with the specified states. Behavior defaults to {@link StateCycleBehavior#DEFAULT}.
     * @param stateMachine - The state machine to cycle through.
     * @param states - The states to cycle through.
     */
    public StateCycle(
        StateMachine<TStateEnum, TPayload> stateMachine,
        List<StateMachine.StateWithPayload<TStateEnum, TPayload>> states
    ) {
        this(stateMachine, states, StateCycleBehavior.DEFAULT);
    }

    /**
     * Creates a new StateCycle from a list of enum states with no payloads.
     * @param <T> The enum type of the states.
     * @param stateMachine - The state machine to cycle through.
     * @param states - The enum states to cycle through.
     * @param behavior - The behavior of the state cycle.
     * @return A new StateCycle instance.
     */
    @SuppressWarnings("unchecked")
    public static <T extends Enum<T>, P> StateCycle<T, P> fromEnumStatesNoPayload(
        StateMachine<T, P> stateMachine,
        List<T> states,
        StateCycleBehavior behavior
    ) {
        return new StateCycle<>(
            stateMachine,
            // Convert T list to StateWithPayload list with null payloads
            (List<StateMachine.StateWithPayload<T, P>>) (List<?>) states
                .stream()
                .map(state -> new StateMachine.StateWithPayload<T, Object>(state, null))
                .toList(),
            behavior
        );
    }

    /**
     * Creates a new StateCycle from a list of enum states with no payloads. Behavior defaults to {@link StateCycleBehavior#DEFAULT}.
     * @param <T> The enum type of the states.
     * @param stateMachine - The state machine to cycle through.
     * @param states - The enum states to cycle through.
     * @return A new StateCycle instance.
     */
    public static <T extends Enum<T>, P> StateCycle<T, P> fromEnumStatesNoPayload(
        StateMachine<T, P> stateMachine,
        List<T> states
    ) {
        return fromEnumStatesNoPayload(stateMachine, states, StateCycleBehavior.DEFAULT);
    }

    /**
     * @return The next state in the cycle depending on the behavior.
     * If the behavior is {@link StateCycleBehavior#RELY_ON_CURRENT_STATE} and the current state is not in the cycle, the optional will be empty.
     * Note: If the state list has duplicates, the first occurrence will be used (for {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}).
     */
    public Optional<StateMachine.StateWithPayload<TStateEnum, TPayload>> getNextState() {
        if (behavior == StateCycleBehavior.RELY_ON_CURRENT_STATE) {
            TStateEnum currentState = stateMachine.getCurrentState().enumType;
            int index = states.indexOf(new StateMachine.StateWithPayload<TStateEnum, Object>(currentState, null));

            // If the current state is not in the cycle, return empty
            if (index == -1) {
                return Optional.empty();
            }

            currentIndex = index;
        }

        // Get the next state in the cycle
        currentIndex = (currentIndex + 1) % states.size();

        try {
            return Optional.of(states.get(currentIndex));
        } catch (IndexOutOfBoundsException e) {
            // Should never happen, but just in case
            e.printStackTrace();
            return Optional.empty();
        }
    }

    /**
     * Schedules the next state in the cycle to be set in the state machine.
     * If the behavior is {@link StateCycleBehavior#RELY_ON_CURRENT_STATE} and the current state is not in the cycle, nothing will be scheduled.
     */
    public void scheduleNextState() {
        Optional<StateMachine.StateWithPayload<TStateEnum, TPayload>> nextState = getNextState();

        nextState.ifPresent(stateMachine::scheduleStateChange);
    }

    /**
     * Resets the state cycle to the given index.
     * For {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}, this will have no effect.
     */
    public void reset(int initialIndex) {
        currentIndex = initialIndex % states.size();
    }

    /**
     * Resets the state cycle to the beginning.
     * For {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}, this will have no effect.
     */
    public void reset() {
        reset(0);
    }
}
