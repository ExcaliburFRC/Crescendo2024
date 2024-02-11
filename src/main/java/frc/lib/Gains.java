package frc.lib;

public class Gains {
    public double kp, ki, kd;
    public double ks, kg, kv, ka;

    private Gains(double kp, double ki, double kd, double ks, double kg, double kv, double ka) {
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
        this(0, 0, 0, ks, kg, kv, ka);
    }

    public Gains(Gains PIDgains, Gains FFgains){
        this(PIDgains.kp, PIDgains.ki, PIDgains.kd, FFgains.ks, FFgains.kg, FFgains.kv, FFgains.ka);
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
