package frc.util.statemachine;

/**
 * State machine with typed payloads for each state.
 * @deprecated This class does not work and has no new use cases. Use StateMachine instead.
 */
public class StateMachineCast<
    TStateEnum extends Enum<TStateEnum> & StateMachineCast.StateEnumWithTypedPayload<TPayload>, TPayload
>
    extends StateMachine<TStateEnum, TPayload> {

    /**
     * Interface for state enums that have typed payloads.
     * @param <TSuperPayload> - The supertype of all payloads for the state enum.
     */
    public interface StateEnumWithTypedPayload<TSuperPayload> {
        /**
         * @return The class of the payload associated with this state enum.
         */
        public Class<? extends TSuperPayload> getPayloadClass();
    }

    /**
     * Creates a new StateMachineCast with the specified dashboard key and state enum class.
     * Identical to {@link StateMachine#StateMachine(String, Class)}.
     */
    public StateMachineCast(Class<TStateEnum> stateEnumClass, String dashboardKey) {
        super(stateEnumClass, dashboardKey);
    }

    public <P extends TPayload> void onStateChangeCastedPayload(
        TStateEnum state,
        StateMachine.OnStateChangeActionNoOptional<TStateEnum, P> action
    ) {
        @SuppressWarnings("unchecked")
        Class<P> payloadClass = (Class<P>) state.getPayloadClass();

        onStateChange(state, payloadClass, action);
    }

    // Override to change return type for method chaining
    @Override
    public StateMachineCast<TStateEnum, TPayload> withState(StateMachineState<TStateEnum> state) {
        super.withState(state);
        return this;
    }

    @Override
    public StateMachineCast<TStateEnum, TPayload> withDefaultState(StateMachineState<TStateEnum> state) {
        super.withDefaultState(state);
        return this;
    }

    @Override
    public StateMachineCast<TStateEnum, TPayload> withReturnToDefaultStateOnDisable(boolean shouldReturn) {
        super.withReturnToDefaultStateOnDisable(shouldReturn);
        return this;
    }
}

// test
class StateMachineCastTest {

    public sealed interface SuperPayload permits PayloadA, PayloadB {}

    public record PayloadA(int value) implements SuperPayload {}

    public record PayloadB(String message) implements SuperPayload {}

    public enum TestStateEnum implements StateMachineCast.StateEnumWithTypedPayload<SuperPayload> {
        STATE_A {
            @Override
            public Class<PayloadA> getPayloadClass() {
                return PayloadA.class;
            }
        },
        STATE_B {
            @Override
            public Class<PayloadB> getPayloadClass() {
                return PayloadB.class;
            }
        },
    }

    public StateMachineCastTest() {
        final StateMachineCast<TestStateEnum, SuperPayload> stateMachine = new StateMachineCast<
            TestStateEnum,
            SuperPayload
        >(TestStateEnum.class, "TestStateMachine")
            .withDefaultState(new StateMachineState<>(TestStateEnum.STATE_A, "StateA"))
            .withState(new StateMachineState<>(TestStateEnum.STATE_B, "StateB"));

        stateMachine.onStateChange(
            TestStateEnum.STATE_A,
            TestStateEnum.STATE_A.getPayloadClass(),
            (stateEnum, payload) -> {
                // System.out.println("Entered STATE_A with value: " + payload.value);
            }
        );
    }
}
