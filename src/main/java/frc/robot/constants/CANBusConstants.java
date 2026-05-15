package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class CANBusConstants {
    public static final CANBus kCANBus = new CANBus("*");

    public static final int kPigeonID = 0;

    public static final int kFrontLeftDriveID = 1;
    public static final int kFrontLeftSteerID = 2;

    public static final int kFrontRightDriveID = 3;
    public static final int kFrontRightSteerID = 4;

    public static final int kBackLeftDriveID = 5;
    public static final int kBackLeftSteerID = 6;

    public static final int kBackRightDriveID = 7;
    public static final int kBackRightSteerID = 8;

    public static final int kIntakeArmID = 9;
    public static final int kIntakeRollerID = 10;

    public static final int kShooterID = 12;
    public static final int kShooterHoodID = 13;

    public static final int kKickerID = 14;

    public static final int kIndexerID = 15;

    public static final int kTurretID = 16;

    public class CANCoder {
        public static final int kFrontLeftSteerID = 21;
        public static final int kFrontRightSteerID = 22;
        public static final int kBackLeftSteerID = 23;
        public static final int kBackRightSteerID = 24;

        public static final int kTurretID = 25;

        public static final int kIntakeID = 26;
    }
}
