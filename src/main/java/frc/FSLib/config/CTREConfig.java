package frc.FSLib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class CTREConfig {

    public static TalonFXConfiguration talonFX () {
        TalonFXConfiguration config = new TalonFXConfiguration();
        return config;
    }

    public static CANcoderConfiguration cancoder (Rotation2d offset) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.MagnetOffset = offset.getRotations();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        return config;
    }
}
