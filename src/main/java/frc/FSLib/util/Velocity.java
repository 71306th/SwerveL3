package frc.FSLib.util;

public class Velocity {
    
    private final double m_x; // meters per second

    public Velocity () {
        this(0.0);
    }

    public Velocity (double value) {
        m_x = value;
    }

    public static Velocity fromMPS (double value) {
        return new Velocity(value);
    }

    public static Velocity fromIPS (double value) {
        return new Velocity(value * 0.0254);
    }

    public double getMPS () {
        return m_x;
    }

    public double getIPS () {
        return m_x * 100 / 2.54;
    }

    public Velocity getAbsloute () {
        return new Velocity(Math.abs(m_x));
    }

    public Velocity plus (Velocity other) {
        return new Velocity(m_x + other.m_x);
    }

    public Velocity minus (Velocity other) {
        return new Velocity(m_x - other.m_x);
    }
    
    public Velocity unaryMinus() {
        return new Velocity(-m_x);
    }

    public Velocity times (double scale) {
        return new Velocity(m_x * scale);
    }

    public Velocity div (double scale) {
        return new Velocity(m_x / scale);
    }

    public double div (Velocity other) {
        return m_x / other.m_x;
    }

    public Velocity toVelocity (double time) {
        return new Velocity(m_x * time);
    }

    public boolean equal_to (Velocity other) {
        return m_x == other.m_x;
    }

    public boolean greater_than (Velocity other) {
        return m_x > other.m_x;
    }

    public boolean abs_greater_than (Velocity other) {
        return Math.abs(m_x) > Math.abs(other.m_x);
    }
    
    public String toString() {
      return String.format("Velocity(meters per second: %.3f)", m_x);
    }

}
