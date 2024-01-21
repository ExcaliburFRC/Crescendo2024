package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterState {
    public double RPM;
    public boolean isLinearOpen;
    public LinearState linearState;

    private static InterpolatingDoubleTreeMap metersToRPM = new InterpolatingDoubleTreeMap();

    static {
        metersToRPM.put(0.0, 0.0);
//        metersToRPM.put(0, 0);
    }

    public ShooterState(double RPM, boolean isLinearOpen) {
        this.RPM = RPM;
        this.isLinearOpen = isLinearOpen;
        linearState = isLinearOpen? LinearState.OPEN : LinearState.CLOSE;
    }

    public ShooterState(double meters) {
        this(metersToRPM.get(meters), false);
    }

    public enum LinearState {
        OPEN(LINEAR_LENGTH),
        CLOSE(0);

        public double length;

        LinearState(double length) {
            this.length = length;
        }
    }
}