package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterConstants.MAX_SHOOTING_RPM;
import static frc.robot.Constants.ShooterConstants.MIN_SHOOTING_RPM;

public class ShooterState {
    private double RPM;
    private boolean IsLinearOpen;
    private InterpolatingDoubleTreeMap Interpolate = new InterpolatingDoubleTreeMap();
    private enum LinearState{
        open(Constants.ShooterConstants.LINEAR_LENGTH),
        close(0);

        LinearState(double linearLength) {
        }
    }

    public ShooterState(double RPM, boolean isLinearOpen) {
        this.RPM = RPM;
        IsLinearOpen = isLinearOpen;
    }

    public ShooterState(double Meters) {
        Interpolate.put(MIN_SHOOTING_RPM ,0.0);
        Interpolate.put(MAX_SHOOTING_RPM,0.0);
        this.RPM = Interpolate.get(Meters);
        this.IsLinearOpen = false;
    }

    public double getRPM() {
        return RPM;
    }

    public boolean isLinearOpen() {
        return IsLinearOpen;
    }
}
