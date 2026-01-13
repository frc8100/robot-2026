package frc.util.statemachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.statemachine.StateCycle.StateCycleBehavior;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

// "An idiot admires complexity, a genius admires simplicity"

/**
 * A generic finite state machine for managing the states of a subsystem.
 * A list of possible states is defined using an enum type.
 * Each enum type can have additional data and behavior defined in a {@link StateMachineState} object.
 * @param <TStateEnum> - The enum type representing all the possible states.
 * @param <TPayload> - The type of payload data associated with each state. When a state is entered, "argument" data of this type can be passed to it.
 */
public class StateMachine<TStateEnum extends Enum<TStateEnum>, TPayload> {

    @FunctionalInterface
    public interface OnStateChangeAction<TStateEnum extends Enum<TStateEnum>, TPayload> {
        /**
         * A functional interface representing an action to perform when a state change occurs.
         * @param previousState - The previous state before the change.
         * @param payload - The payload associated with the new state.
         */
        public void onStateChange(Optional<TStateEnum> previousState, Optional<TPayload> payload);
    }

    @FunctionalInterface
    public interface OnStateChangeActionNoOptional<TStateEnum extends Enum<TStateEnum>, TPayload> {
        /**
         * A functional interface representing an action to perform when a state change occurs.
         * Does not use Optional for the payload.
         * @param previousState - The previous state before the change.
         * @param payload - The payload associated with the new state.
         */
        public void onStateChange(Optional<TStateEnum> previousState, TPayload payload);
    }

    @FunctionalInterface
    public interface StatePeriodicAction<TPayload> {
        /**
         * A functional interface representing an action to perform periodically while in a state.
         * @param payload - The payload associated with the current state.
         */
        public void onPeriodic(Optional<TPayload> payload);
    }

    @FunctionalInterface
    public interface StatePeriodicActionNoOptional<TPayload> {
        /**
         * A functional interface representing an action to perform periodically while in a state.
         * Does not use Optional for the payload.
         * @param payload - The payload associated with the current state.
         */
        public void onPeriodic(TPayload payload);
    }

    /**
     * A record representing a state along with its associated payload.
     * @param <TStateEnum> - The enum type representing the state.
     * @param <TPayload> - The type of payload associated with the state.
     */
    public static record StateWithPayload<TStateEnum extends Enum<TStateEnum>, TPayload>(
        TStateEnum state,
        TPayload payload
    ) {
        /**
         * @return Whether this StateWithPayload is equal to another object.
         * If the other object is a StateWithPayload, they are equal if their states are equal.
         * Note: payloads are not considered for equality.
         */
        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;

            StateWithPayload<?, ?> other = (StateWithPayload<?, ?>) obj;

            return state.equals(other.state);
        }
    }

    /**
     * The prefix key for logging to the dashboard.
     */
    private static final String DEFAULT_DASHBOARD_KEY = "StateMachines/";

    /**
     * A trigger that is active when the driver station/robot is disabled.
     */
    private static final Trigger onDriverStationDisable = new Trigger(DriverStation::isDisabled);

    /**
     * The current state of the subsystem.
     * Can be null if no state has been set yet.
     */
    private StateMachineState<TStateEnum> currentState = null;

    /**
     * The payload associated with the current state.
     * Can be null.
     */
    private TPayload currentPayload = null;

    /**
     * The payload associated with the current state, wrapped in an Optional for caching.
     */
    private Optional<TPayload> currentPayloadOptional = Optional.empty();

    /**
     * The default state of the subsystem.
     * Used to initialize the current state if it is null.
     * Also used to reset the state machine to its default state.
     */
    private StateMachineState<TStateEnum> defaultState = null;

    /**
     * The key used for logging to the dashboard.
     * Example: "StateMachines/Swerve"
     */
    public final String dashboardKey;

    /**
     * The class of the enum type representing the states of the state machine.
     */
    private final Class<TStateEnum> stateEnumClass;

    /**
     * A map from enum states to their corresponding {@link StateMachineState} objects.
     */
    private final Map<TStateEnum, StateMachineState<TStateEnum>> stateMap;

    /**
     * A map of actions to perform on state changes.
     * When the state machine changes to a state, all actions in the list for that state are executed once.
     */
    private final Map<TStateEnum, List<OnStateChangeAction<TStateEnum, TPayload>>> onStateChangeActions;

    /**
     * A map from enum states to their corresponding actions to perform each period while in that state.
     */
    public final Map<TStateEnum, StatePeriodicAction<TPayload>> statePeriodicActions;

    /**
     * The state that has been scheduled to change to on the next update cycle. Can be null.
     * See {@link #scheduleStateChange}.
     */
    private StateMachineState<TStateEnum> scheduledStateChange = null;

    /**
     * The payload associated with the current {@link #scheduledStateChange}. Can be null.
     */
    private TPayload scheduledPayload = null;

    /**
     * Whether the state machine should return to the default state when disabled.
     */
    private boolean shouldReturnToDefaultStateOnDisable = false;

    /**
     * Constructs a state machine with the specified initial state.
     * State is logged to the dashboard with the given key.
     * @param stateEnumClass - The class of the enum type representing the states of the state machine. Used to initialize the internal {@link EnumMap}s. Ex. {@code MyStateEnum.class}
     * @param dashboardKey - The key to use for logging to the dashboard. Added as a prefix to {@link #DEFAULT_DASHBOARD_KEY}.
     */
    public StateMachine(Class<TStateEnum> stateEnumClass, String dashboardKey) {
        this.dashboardKey = DEFAULT_DASHBOARD_KEY + dashboardKey;
        this.stateEnumClass = stateEnumClass;

        // Init maps
        stateMap = new EnumMap<>(this.stateEnumClass);
        onStateChangeActions = new EnumMap<>(this.stateEnumClass);
        statePeriodicActions = new EnumMap<>(this.stateEnumClass);

        // Initialize action lists for each state
        for (TStateEnum state : this.stateEnumClass.getEnumConstants()) {
            onStateChangeActions.put(state, new ArrayList<>());
        }

        recordCurrentState();
        recordScheduledStateChange();

        // Schedule state machine updates
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::commandSchedulerLoop);

        // Disable handling
        onDriverStationDisable.onTrue(Commands.runOnce(this::disableHandle).ignoringDisable(true));
    }

    /**
     * Handles returning to the default state when the robot is disabled, if enabled.
     */
    private void disableHandle() {
        // Return to default state if enabled and not already in default state
        if (
            shouldReturnToDefaultStateOnDisable &&
            defaultState != null &&
            (currentState == null || !currentState.equals(defaultState))
        ) {
            System.out.println(
                "[StateMachine \"" +
                dashboardKey +
                "\"] Returning to default state on disable: " +
                defaultState.enumType
            );

            setStateAndUpdate(defaultState.enumType);

            scheduledStateChange = null;
            recordScheduledStateChange();
        }
    }

    /**
     * A function that runs every command scheduler cycle to handle scheduled state changes.
     */
    private void commandSchedulerLoop() {
        // Check if the scheduled state can be changed to the new state based on the requirements
        if (scheduledStateChange == null) return;
        if (!scheduledStateChange.canChangeCondition.canChange(currentState.enumType)) return;

        setStateAndUpdate(new StateWithPayload<>(scheduledStateChange.enumType, scheduledPayload));

        scheduledStateChange = null;
        scheduledPayload = null;
        recordScheduledStateChange();
    }

    /**
     * @param state - The enum state to get the corresponding StateMachineState object for.
     * @return The StateMachineState object corresponding to the given enum state.
     * @throws IllegalArgumentException if the state does not exist in the state machine.
     */
    private StateMachineState<TStateEnum> getStateObject(TStateEnum state) {
        if (!stateMap.containsKey(state)) {
            throw new IllegalArgumentException("StateMachine does not contain state: " + state);
        }

        return stateMap.get(state);
    }

    /**
     * Executes all actions registered for the current state change.
     * @param previousState - The previous state before the change. Can be null if there was no previous state.
     */
    private void executeOnStateChangeActions(TStateEnum previousState) {
        if (currentState == null) return;

        List<OnStateChangeAction<TStateEnum, TPayload>> actions = onStateChangeActions.get(currentState.enumType);

        for (var action : actions) {
            action.onStateChange(Optional.ofNullable(previousState), getCurrentPayload());
        }
    }

    /**
     * Records the current state to the dashboard.
     */
    private void recordCurrentState() {
        if (currentState == null) {
            Logger.recordOutput(dashboardKey + "/State", "None");
            return;
        }

        Logger.recordOutput(dashboardKey + "/State", currentState.toString());
    }

    /**
     * Records the scheduled state change to the dashboard.
     */
    private void recordScheduledStateChange() {
        if (scheduledStateChange == null) {
            Logger.recordOutput(dashboardKey + "/ScheduledStateChange", "None");
            return;
        }

        Logger.recordOutput(dashboardKey + "/ScheduledStateChange", scheduledStateChange.toString());
    }

    /**
     * Sets the current state of the state machine, runs the onStateChange actions, and records the state change.
     * Bypasses any transition requirements.
     * @param newState - The new state to set, as the enum type.
     */
    private void setStateAndUpdate(TStateEnum newState) {
        TStateEnum previousState = (currentState != null) ? currentState.enumType : null;

        currentState = getStateObject(newState);
        recordCurrentState();

        executeOnStateChangeActions(previousState);
    }

    private void setStateAndUpdate(StateWithPayload<TStateEnum, TPayload> stateWithPayload) {
        currentPayload = stateWithPayload.payload;
        currentPayloadOptional = Optional.ofNullable(currentPayload);
        setStateAndUpdate(stateWithPayload.state);
    }

    /**
     * Sets the current state of the state machine.
     * @param newState - The new state to set, as the enum type.
     * @return True if the state was changed, false if it was the same as the current state.
     * @throws IllegalArgumentException if the new state does not exist in the state machine.
     */
    public boolean setState(TStateEnum newState) {
        // Check if the new state is the same as the current state
        if (newState == getCurrentState().enumType) {
            return false;
        }

        StateMachineState<TStateEnum> newStateObj = getStateObject(newState);

        // Check if the state can be changed to the new state based on the requirements
        if (!newStateObj.canChangeCondition.canChange(currentState.enumType)) {
            return false;
        }

        setStateAndUpdate(newState);

        return true;
    }

    public boolean setState(StateWithPayload<TStateEnum, TPayload> stateWithPayload) {
        // Check if the new state is the same as the current state
        if (stateWithPayload.state == getCurrentState().enumType) {
            return false;
        }

        StateMachineState<TStateEnum> newStateObj = getStateObject(stateWithPayload.state);

        // Check if the state can be changed to the new state based on the requirements
        if (!newStateObj.canChangeCondition.canChange(currentState.enumType)) {
            return false;
        }

        setStateAndUpdate(stateWithPayload);

        return true;
    }

    /**
     * Forces the current state of the state machine to the specified state.
     * ! IMPORTANT: Bypasses any transition requirements.
     * @param newState - The new state to set, as the enum type.
     * @throws IllegalArgumentException if the new state does not exist in the state machine.
     */
    public void forceSetState(TStateEnum newState) {
        setStateAndUpdate(newState);
    }

    /**
     * Schedules a state change to the specified state on the next update cycle.
     * If the state can be changed immediately, it will be changed immediately and this will return true.
     * @param newState - The new state to change to, as the enum type.
     * @return Whether the state change was changed immediately.
     * @throws IllegalArgumentException if the new state does not exist in the state machine.
     */
    public boolean scheduleStateChange(TStateEnum newState) {
        StateMachineState<TStateEnum> newStateObj = getStateObject(newState);

        // If it can change now, change immediately
        if (newStateObj.canChangeCondition.canChange(currentState.enumType)) {
            setStateAndUpdate(newState);
            return true;
        }

        scheduledStateChange = newStateObj;
        recordScheduledStateChange();
        return false;
    }

    public boolean scheduleStateChange(StateWithPayload<TStateEnum, TPayload> stateWithPayload) {
        StateMachineState<TStateEnum> newStateObj = getStateObject(stateWithPayload.state);

        // If it can change now, change immediately
        if (newStateObj.canChangeCondition.canChange(currentState.enumType)) {
            setStateAndUpdate(stateWithPayload);
            return true;
        }

        scheduledStateChange = newStateObj;
        scheduledPayload = stateWithPayload.payload;
        recordScheduledStateChange();
        return false;
    }

    public Command scheduleStateChangeCommand(TStateEnum newState) {
        if (scheduleStateChange(newState)) {
            // Finished immediately
            return Commands.none();
        } else {
            return Commands.waitUntil(
                () -> scheduledStateChange == null && currentState != null && currentState.enumType == newState
            );
        }
    }

    /**
     * Unschedule any scheduled state change.
     */
    public void unscheduleStateChange() {
        scheduledStateChange = null;
        recordScheduledStateChange();
    }

    /**
     * @return Whether any states are currently scheduled.
     */
    public boolean isAnyStateScheduled() {
        return scheduledStateChange != null;
    }

    /**
     * Resets the state machine to its default state.
     * ! IMPORTANT: Bypasses any transition requirements.
     */
    public void reset() {
        if (defaultState == null) return;

        currentState = defaultState;
    }

    /**
     * Adds a state to the state machine.
     * @param state - The state to add.
     */
    public void addState(StateMachineState<TStateEnum> state) {
        stateMap.put(state.enumType, state);
    }

    /**
     * Adds a state to the state machine and returns the state machine for chaining.
     * @param state - The state to add.
     */
    public StateMachine<TStateEnum, TPayload> withState(StateMachineState<TStateEnum> state) {
        addState(state);
        return this;
    }

    /**
     * Sets the default state of the state machine.
     * Also sets the current state to the default state if the current state is null.
     * @param defaultState - The default state to set.
     */
    public void setDefaultState(StateMachineState<TStateEnum> defaultState) {
        this.defaultState = defaultState;

        // If currentState is null, set it to the default state
        if (currentState == null) {
            setStateAndUpdate(defaultState.enumType);
        }
    }

    /**
     * Sets and adds the default state of the state machine and returns the state machine for chaining.
     * @param defaultState - The default state to set.
     */
    public StateMachine<TStateEnum, TPayload> withDefaultState(StateMachineState<TStateEnum> defaultState) {
        addState(defaultState);
        setDefaultState(defaultState);
        return this;
    }

    /**
     * Sets whether the state machine should return to the default state when disabled.
     * @param shouldReturn - Whether the state machine should return to the default state when disabled.
     */
    public void setReturnToDefaultStateOnDisable(boolean shouldReturn) {
        this.shouldReturnToDefaultStateOnDisable = shouldReturn;
    }

    /**
     * Sets whether the state machine should return to the default state when disabled, and returns the state machine for chaining.
     * @param shouldReturn - Whether the state machine should return to the default state when disabled.
     */
    public StateMachine<TStateEnum, TPayload> withReturnToDefaultStateOnDisable(boolean shouldReturn) {
        setReturnToDefaultStateOnDisable(shouldReturn);
        return this;
    }

    /**
     * @return The current state of the state machine.
     * @throws IllegalStateException if the state machine does not have a current state defined and no default state to fall back on.
     */
    public StateMachineState<TStateEnum> getCurrentState() {
        if (currentState == null) {
            // Set to default state if it exists
            // This should never happen if the state machine is properly initialized
            if (defaultState != null) {
                setStateAndUpdate(defaultState.enumType);
            } else {
                throw new IllegalStateException("StateMachine does not have a current state defined.");
            }
        }

        return currentState;
    }

    /**
     * @return The payload associated with the current state, if any.
     */
    public Optional<TPayload> getCurrentPayload() {
        return currentPayloadOptional;
    }

    /**
     * Attempts to get the current payload as the specified class.
     * Useful for state machines specifying payload types with values that are only used in certain states.
     * The {@link TPayload} type can be a common superclass or interface of multiple payload types.
     * This method can cast that common type to a more specific type (that is a subclass or implementation of the common type).
     * @param <T> - The class type to cast the payload to.
     * @param payloadClass - The class to cast the payload to. Ex. {@code MyPayloadClass.class}
     * @return An Optional containing the payload cast to the specified class, or an empty Optional if the payload is null or cannot be cast.
     */
    public <T extends TPayload> Optional<T> getCurrentPayloadAs(Class<T> payloadClass) {
        if (currentPayload == null) {
            return Optional.empty();
        }

        if (!payloadClass.isInstance(currentPayload)) {
            return Optional.empty();
        }

        return Optional.of(payloadClass.cast(currentPayload));
    }

    /**
     * Adds an action to perform when the state machine changes to the specified state.
     * @param state - The state to add the action for.
     * @param action - The action to perform when the state machine changes to the specified state.
     * @throws IllegalArgumentException if the state does not exist in the state machine.
     */
    public void onStateChange(TStateEnum state, OnStateChangeAction<TStateEnum, TPayload> action) {
        if (!onStateChangeActions.containsKey(state)) {
            throw new IllegalArgumentException("StateMachine does not contain state: " + state);
        }

        onStateChangeActions.get(state).add(action);
    }

    /**
     * Adds an action to perform when the state machine changes to the specified state.
     * If the payload is null or cannot be cast, the action is not performed.
     * @param <T> - The class type to cast the payload to.
     * @param state - The state to add the action for.
     * @param payloadClass - The class to cast the payload to. Ex. {@code MyPayloadClass.class}
     * @param action - The action to perform when the state machine changes to the specified state.
     */
    public <T extends TPayload> void onStateChange(
        TStateEnum state,
        Class<T> payloadClass,
        OnStateChangeActionNoOptional<TStateEnum, T> action
    ) {
        onStateChange(state, (previousState, payload) -> {
            if (payloadClass.isInstance(payload.orElse(null))) {
                action.onStateChange(previousState, payloadClass.cast(payload.get()));
            } else {
                // Log warning about payload type mismatch
                DriverStation.reportWarning(
                    "[StateMachine \"" +
                    dashboardKey +
                    "\"] onStateChange action for state " +
                    state +
                    " skipped due to payload type mismatch or null payload. Expected: " +
                    payloadClass.getName() +
                    " but got: " +
                    (payload.isPresent() ? payload.get().getClass().getName() : "null"),
                    false
                );
            }
        });
    }

    /**
     * Adds an action to perform when the state machine changes to the specified state.
     * Does not use the payload.
     * @param state - The state to add the action for.
     * @param action - The action to perform when the state machine changes to the specified state.
     * @throws IllegalArgumentException if the state does not exist in the state machine.
     */
    public void onStateChange(TStateEnum state, Runnable action) {
        onStateChange(state, (previousState, payload) -> action.run());
    }

    /**
     * Adds an action to perform each period while the state machine is in the specified state.
     * @param state - The state to add the action for.
     * @param action - The action to perform each period while the state machine is in the specified state.
     */
    public void whileState(TStateEnum state, StatePeriodicAction<TPayload> action) {
        statePeriodicActions.put(state, action);
    }

    /**
     * Adds an action to perform each period while the state machine is in the specified state.
     * If the payload is null or cannot be cast, the action is not performed and skipped for that period.
     * @param <T> - The class type to cast the payload to.
     * @param state - The state to add the action for.
     * @param payloadClass - The class to cast the payload to. Ex. {@code MyPayloadClass.class}
     * @param action - The action to perform each period while the state machine is in the specified state.
     */
    public <T extends TPayload> void whileState(
        TStateEnum state,
        Class<T> payloadClass,
        StatePeriodicActionNoOptional<T> action
    ) {
        whileState(state, payload -> {
            if (payloadClass.isInstance(payload.orElse(null))) {
                action.onPeriodic(payloadClass.cast(payload.get()));
            }
        });
    }

    /**
     * Adds an action to perform each period while the state machine is in the specified state.
     * Does not use the payload.
     * @param state - The state to add the action for.
     * @param action - The action to perform each period while the state machine is in the specified state.
     */
    public void whileState(TStateEnum state, Runnable action) {
        whileState(state, payload -> action.run());
    }

    /**
     * @param requirements - The subsystems required by the returned command.
     * @return A command that runs the state machine's current state's action each period.
     */
    public StateMachineSubsystemCommand<TStateEnum, TPayload> getRunnableCommand(Subsystem... requirements) {
        return new StateMachineSubsystemCommand<>(this, requirements);
    }

    /**
     * Checks if the state machine is currently in the specified state.
     * @param state - The state to check.
     * @return True if the state machine is in the specified state, false otherwise.
     */
    public boolean is(TStateEnum state) {
        return getCurrentState().enumType == state;
    }

    /**
     * Returns a trigger that is active when the state machine is in the specified state.
     * See {@link #addOnStateChangeAction} for an better way to run actions on state changes.
     * @param state - The state to create a trigger for.
     */
    public Trigger on(TStateEnum state) {
        return new Trigger(() -> is(state));
    }

    /**
     * Creates a StateCycle with the specified states and behavior.
     * @param states - The states to cycle through.
     * @param behavior - The behavior of the state cycle.
     * @return The created StateCycle.
     */
    public StateCycle<TStateEnum, TPayload> createStateCycle(
        List<TStateEnum> states,
        StateCycle.StateCycleBehavior behavior
    ) {
        return StateCycle.fromEnumStatesNoPayload(this, states, behavior);
    }

    /**
     * Creates a StateCycle with the specified states. Behavior defaults to {@link StateCycleBehavior#RELY_ON_CURRENT_STATE}.
     * @param states - The states to cycle through.
     * @return The created StateCycle.
     */
    public StateCycle<TStateEnum, TPayload> createStateCycle(List<TStateEnum> states) {
        return StateCycle.fromEnumStatesNoPayload(this, states);
    }

    public StateCycle<TStateEnum, TPayload> createStateCycleWithPayload(
        List<StateWithPayload<TStateEnum, TPayload>> states,
        StateCycle.StateCycleBehavior behavior
    ) {
        return new StateCycle<>(this, states, behavior);
    }

    public StateCycle<TStateEnum, TPayload> createStateCycleWithPayload(
        List<StateWithPayload<TStateEnum, TPayload>> states
    ) {
        return new StateCycle<>(this, states);
    }
}
