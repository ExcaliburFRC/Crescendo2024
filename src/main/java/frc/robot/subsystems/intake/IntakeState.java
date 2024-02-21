package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeState {
    public double intakeDC;
    public IntakeAngle angle;
    public boolean waitForAngle;

    public enum IntakeAngle {
        GROUND(-22),
        AMP(104),
        HUMAN_PLAYER_BACKWARD(146),
        HUMAN_PLAYER_FORWARD(104), // needs to intake from shooter as well
        SHOOTER(168);

        public final int angle;

        IntakeAngle(int angle) {
            this.angle = angle + INTAKE_READING_OFFSET;
        }
    }

    public IntakeState(double dc, IntakeAngle angle, boolean waitForAngle) {
        this.intakeDC = dc;
        this.angle = angle;
        this.waitForAngle = waitForAngle;
    }
}
