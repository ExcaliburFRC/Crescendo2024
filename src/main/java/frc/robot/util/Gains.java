package frc.robot.util;

public class Gains {
    public double kp, ki, kd;
    public double ks, kg, kv, ka;

    public Gains(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
    }

    public Gains(double kp, double ki, double kd) {
        this(kp, ki, kd, 0, 0, 0, 0);
    }

    public Gains(double ks, double kg, double kv, double ka) {
        this(0, 0, 0, ks, kv, ka, kg);
    }

    public Gains(Gains PIDgains, Gains FFgains){
        this(PIDgains.kp, PIDgains.ki, PIDgains.kd, FFgains.ks, FFgains.kv, FFgains.ka, FFgains.kg);
    }

    public Gains(){
        this(0, 0, 0, 0, 0, 0, 0);
    }

    public Gains(Gains gains){
        this(gains.kp, gains.ki, gains.kd, gains.ks, gains.kg, gains.kv, gains.ka);
    }

    public void setPIDgains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setFeedForwardGains(double ks, double kg, double kv, double ka) {
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
    }
}
