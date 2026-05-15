package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class IndexerConstants {
    public static final double kMaxVoltage = 10.0;
    public static final double kMinVoltage = 5.0;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
}
