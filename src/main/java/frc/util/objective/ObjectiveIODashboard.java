package frc.util.objective;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ObjectiveIODashboard implements ObjectiveIO {

    @Override
    public void updateInputs(ObjectiveIOInputs inputs) {
        if (DriverStation.isAutonomous()) {
            inputs.activeHub = ObjectiveIO.ActiveHub.ALL;
        }

        // todo: update from https://gist.github.com/OliverW1986/f5910996381601e32cf475bf261f456d
        double timeUntilNextPeriodSeconds = DriverStation.getMatchTime();
    }
}
