package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.0;

    public static final double kDiscontinuityPoint = 0.0001;
    public static final double kMagnetOffset = 0.0;

    public static final double kRotorOffset = 0.0;

    public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue kSensorDirectionValue = SensorDirectionValue.Clockwise_Positive;

    public static final double kGearRatio = 1.0;

    public static final double kMaxAngle = 2.0;
    public static final double kMinAngle = -2.0;

    public static final double kPositionTolerance = 0.05;

    public static final Translation2d kTurretOffset = new Translation2d(0.0, 0.0);
}
