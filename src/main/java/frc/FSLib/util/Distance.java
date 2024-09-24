package frc.FSLib.util;

public class Distance {

    private final double m_x; // meters
    
    public Distance () {
        this(0.0);
    }

    public Distance (double value) {
        m_x = value;
    }

    public static Distance fromMeters (double value) {
        return new Distance(value);
    }

    public static Distance fromMillimeters (double value) {
        return new Distance(value * 0.001);
    }

    public static Distance fromInches (double value) {
        return new Distance(value * 0.0254);
    }

    public double getMeters () {
        return m_x;
    }

    public double getMilliters () {
        return m_x * 1000;
    }

    public double getInches () {
        return m_x * 100 / 2.54;
    }

    public Distance getAbsloute () {
        return new Distance(Math.abs(m_x));
    }

    public Distance plus (Distance other) {
        return new Distance(m_x + other.m_x);
    }

    public Distance minus (Distance other) {
        return new Distance(m_x - other.m_x);
    }

    public Distance times (double scale) {
        return new Distance(m_x * scale);
    }

    public Distance div (double scale) {
        return new Distance(m_x / scale);
    }

    public boolean equal_to (Distance other) {
        return m_x == other.m_x;
    }

    public boolean greater_than (Distance other) {
        return m_x > other.m_x;
    }

    public boolean abs_greater_than (Distance other) {
        return Math.abs(m_x) > Math.abs(other.m_x);
    }

    public String toString () {
        return String.format("Distance:(Meters: %3.f)", m_x);
    }

}
