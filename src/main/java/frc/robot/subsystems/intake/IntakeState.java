package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeState {
    public double dc;
    public intakeAngle angle;
    public boolean waitForAngle;

    public enum intakeAngle {
        GROUND(0),
        AMP(0),
        HUMAN_PLAYER(0),
        SHOOTER(0);

        public final int angle;

        intakeAngle(int angle) {
            if (angle < MINIMAL_INTAKE_ANGLE) throw new IllegalArgumentException("intake angle lower then mechanical stop");
            this.angle = angle;
        }
    }

    public IntakeState(double dc, intakeAngle angle, boolean waitForAngle) {
        this.dc = dc;
        this.angle = angle;
        this.waitForAngle = waitForAngle;
    }
}
