package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import frc.robot.generated.TunerConstants;

public final class DriverConstants {
        public final static int kDriver = 0;
        public final static int kOperator = 1;

        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double kMaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
        public static final double kDriveDeadDand = 0;
        public static final double kRotationDeadBand = 0;
}