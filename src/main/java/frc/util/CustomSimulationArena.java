package frc.util;

import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * Custom simulation arena that extends the 2026 rebuilt arena.
 * Exposes the physics world for testing purposes.
 */
public class CustomSimulationArena extends Arena2026Rebuilt {

    public CustomSimulationArena(boolean addRampCollider) {
        super(addRampCollider);
    }

    public World<Body> getPhysicsWorld() {
        return super.physicsWorld;
    }
}
