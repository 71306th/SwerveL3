package frc.FSLib.util;

public class AngularVelocity {
    
    private final double m_x; // rads per second
    private final double m_r; // meters per rad;

    public AngularVelocity () {
        this(1.0, 1.0);
    }

    public AngularVelocity (double value) {
        this(value, 1.0);
    }

    public AngularVelocity (double value, double ratio) {
        m_x = value;
        m_r = ratio;
    }

    public static AngularVelocity fromRadPS (double value) {
        return new AngularVelocity(value);
    }

    public static AngularVelocity fromRadPM (double value) {
        return new AngularVelocity(value / 60);
    }

    public static AngularVelocity fromRevPS (double value) {
        return new AngularVelocity(value * 2 * Math.PI);
    }

    public static AngularVelocity fromRevPM (double value) {
        return new AngularVelocity(value * Math.PI / 30);
    }

    public static AngularVelocity fromDegPS (double value) {
        return new AngularVelocity(value * Math.PI / 180);
    }

    public static AngularVelocity fromDegPM (double value) {
        return new AngularVelocity(value * Math.PI / 7200);
    }

    public double getRadPS () {
        return m_x;
    }

    public double getRadPM () {
        return m_x * 60;
    }

    public double getRevPS () {
        return m_x * 0.5 / Math.PI;
    }

    public double getRevPM () {
        return m_x * 30 / Math.PI;
    }

    public double getDegPS () {
        return m_x * 180 / Math.PI;
    }

    public double getDegPM () {
        return m_x * 1080 / Math.PI;
    }

    public AngularVelocity getAbsloute () {
        return new AngularVelocity(Math.abs(m_x), m_r);
    }

    public AngularVelocity plus (AngularVelocity other) throws Exception {
        if (other.m_r != m_r) throw new Exception("the ratio of AngularVelocity is different");
        return new AngularVelocity(m_x + other.m_x, m_r);
    }

    public AngularVelocity minus (AngularVelocity other) throws Exception {
        if (other.m_r != m_r) throw new Exception("the ratio of AngularVelocity is different");
        return new AngularVelocity(m_x - other.m_x, m_r);
    }

    public AngularVelocity times (double scale) {
        return new AngularVelocity(m_x * scale, m_r);
    }

    public AngularVelocity div (double scale) {
        return new AngularVelocity(m_x / scale, m_r);
    }

    public Velocity toVelocity () {
        return new Velocity(m_x * m_r);
    }

    public Velocity toVelocity (double ratio) {
        return new Velocity(m_x * ratio);
    }

    public boolean equal_to (AngularVelocity other) {
        return m_x == other.m_x;
    }

    public boolean greater_than (AngularVelocity other) {
        return m_x > other.m_x;
    }

    public boolean abs_greater_than (AngularVelocity other) {
        return Math.abs(m_x) > Math.abs(other.m_x);
    }

    public String toString() {
        return String.format("AngularVelocity(rad/s: %.3f, ratio: %.3f",m_x,m_r);
    }

}
