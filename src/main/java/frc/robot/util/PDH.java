package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import monologue.Annotations.Log;
import monologue.Logged;

public class PDH implements Logged {
    private final PowerDistribution pdh = new PowerDistribution();

    private final int PDH_CHANNELS = 23;

    private double[] portsAmp = new double[PDH_CHANNELS];

    @Log.NT
    private double totalCurrent(){
        return pdh.getTotalCurrent();
    }

    @Log.NT
    private double totalWatts(){
        return pdh.getTotalPower();
    }

    @Log.NT
    private double totalPowerEnergy(){
        return pdh.getTotalEnergy();
    }

    @Log.NT
    private double[] portsAmps(){
        for (int i = 0; i < PDH_CHANNELS; i++) {
            portsAmp[i] = pdh.getCurrent(i);
        }

        return portsAmp;
    }

    @Log.NT
    private double temperature(){
        return pdh.getTemperature();
    }
}
