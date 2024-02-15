package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeState {
    public double intakeDC;
    public IntakeAngle angle;
    public boolean waitForAngle;

    public enum IntakeAngle {
        GROUND(343),
        AMP(104),
        HUMAN_PLAYER(104),
        SHOOTER(130);

        public final int angle;

        IntakeAngle(int angle) {
            if (angle < MINIMAL_INTAKE_ANGLE) throw new IllegalArgumentException("intake angle lower then mechanical stop");
            this.angle = angle;
        }
    }

    public IntakeState(double dc, IntakeAngle angle, boolean waitForAngle) {
        this.intakeDC = dc;
        this.angle = angle;
        this.waitForAngle = waitForAngle;
    }
}
