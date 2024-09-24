package frc.FSLib.math;

public class NoiseFilter {

    private final int m_l;
    private final double[] m_x;

    private int m_ptr = 0;
    private double m_sum = 0;

    public NoiseFilter () {
        this(20);
    }

    public NoiseFilter (int bufferSize) {
        m_l = bufferSize;
        m_x = new double[bufferSize];
        for (int i = 0 ; i < bufferSize ; i++) {
            m_x[i] = 0;
        }
    }

    public double getAverage (double newValue) {
        m_sum = m_sum - m_x[m_ptr] + newValue;
        m_x[m_ptr] = newValue;
        m_ptr = m_ptr == m_l-1 ? 0 : ++m_ptr;
        return m_sum / m_l;
    }

}
