package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeState {
    public double intakeDC;
    public intakeAngle angle;
    public boolean waitForAngle;

    public enum intakeAngle {
        GROUND(343),
        AMP(104),
        HUMAN_PLAYER(104),
        SHOOTER(130);

        public final int angle;

        intakeAngle(int angle) {
            if (angle < MINIMAL_INTAKE_ANGLE) throw new IllegalArgumentException("intake angle lower then mechanical stop");
            this.angle = angle;
        }
    }

    public IntakeState(double dc, intakeAngle angle, boolean waitForAngle) {
        this.intakeDC = dc;
        this.angle = angle;
        this.waitForAngle = waitForAngle;
    }
}
