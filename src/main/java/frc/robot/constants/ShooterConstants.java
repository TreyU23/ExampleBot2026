package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class ShooterConstants {
    public class Shooter {
        public static final double kP = 50.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
    }

    public class Hood {
        public static final double kP = 50.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kGearRatio = 0.0;

        public static final double kPostionTolerance = 0.05;

        public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
    }
}