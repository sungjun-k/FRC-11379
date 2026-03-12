package frc.robot;

public final class Constants {

    public static final class DriveConstants {
        public static final int LEFT_FRONT_ID  = 1;
        public static final int LEFT_REAR_ID   = 2;
        public static final int RIGHT_FRONT_ID = 3;
        public static final int RIGHT_REAR_ID  = 4;

        public static final double DRIVE_SPEED_SCALE    = 0.6;
        public static final double DRIVE_ROTATION_SCALE = 0.35;
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

        public static final double WHEEL_DIAMETER_METERS = 0.1524;
        public static final double GEAR_RATIO             = 5.95;
        public static final double TRACK_WIDTH_METERS     = 0.561;
        public static final double ENCODER_POSITION_FACTOR =
            (Math.PI * WHEEL_DIAMETER_METERS) / GEAR_RATIO;
        public static final double MAX_SPEED_MPS = 5.0;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT   = 0;  // 드라이버 (송주원)
        public static final int OPERATOR_CONTROLLER_PORT = 1;  // 오퍼레이터 (김도윤)
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_5_ID = 5;
        public static final int SHOOTER_MOTOR_6_ID = 6;
        public static final int SHOOTER_MOTOR_7_ID = 7;
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_ROLLER_ID = 8;
        public static final int INTAKE_PIVOT_ID  = 9;

        public static final double ROLLER_SPEED = 0.8;
        public static final boolean ROLLER_INVERTED = true;
        public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

        public static final double PIVOT_CURRENT_LIMIT_AMPS           = 25.0;
        public static final double PIVOT_SECONDARY_CURRENT_LIMIT_AMPS = 30.0;

        public static final double PIVOT_KP = 0.08;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.005;

        public static final double PIVOT_KG_VOLTS = 0.5;
        public static final double PIVOT_HOME_ANGLE_DEG = 90.0;
        public static final double PIVOT_GEAR_RATIO = 80.0;

        public static final double PIVOT_HOME_POS   = 0.0;
        public static final double PIVOT_INTAKE_POS = -11;
        public static final double PIVOT_BUMP_STEP  = 0.3;

        public static final float PIVOT_FORWARD_SOFT_LIMIT = 1.0f;
        public static final float PIVOT_REVERSE_SOFT_LIMIT = -12f;
    }

    public static final class ConveyorConstants {
        public static final int    CONVEYOR_MOTOR_ID          = 10;
        public static final double CONVEYOR_SPEED             = 1.0;
        public static final boolean CONVEYOR_INVERTED         = false;
        public static final double CONVEYOR_CURRENT_LIMIT_AMPS = 35.0;

        // 패턴 타이멑
        public static final double CONVEYOR_RUN_TIME  = 0.15; // 0.15초 ON
        public static final double CONVEYOR_STOP_TIME = 0.40; // 0.40초 OFF
    }
}
