package frc.lib;

public class Gains {
    public double kp, ki, kd;
    public double ks, kg, kv, ka;
    public static final Gains EMPTY_GAINS = new Gains(0,0,0);

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
    public static Gains copyGains(Gains gains){
        if (gains == null) return EMPTY_GAINS;
        return new Gains(gains.kp, gains.ki, gains.kd, gains.ks, gains.kg, gains.kv, gains.ka);
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
