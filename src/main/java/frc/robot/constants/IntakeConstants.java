package frc.robot.constants;

public class IntakeConstants {
    public static final double kMagnetOffset = 0.0;

    public class Arm {
        public static final double kMaxPosition = 90.0;
        public static final double kMinPosition = 0.0;

        public static final double kIntakePosition = 0.0;
        public static final double kRestingPosition = 60.0;

        public static final double kGearRatio = 25.0/1.0;

        public static final double kPositionTolerance = 0.05;

            public static final double kP = 20.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.30;
            public static final double kG = -0.20;
    }

    public class Roller {
        public static final double kMaxVoltage = 10.0;
        public static final double kMinVoltage = 5.0;
    }

}
