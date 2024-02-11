package frc.robot.util;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class SysIdConfig extends SysIdRoutine.Config {
    public SysIdConfig(double rampRate, double stepVoltage, double timeout){
        super(Volts.of(rampRate).per(Seconds.of(1.0)), Volts.of(stepVoltage), Seconds.of(timeout));
    }
}
