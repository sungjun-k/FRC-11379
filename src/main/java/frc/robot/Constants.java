package frc.robot;

public final class Constants {

    public static final class DriveConstants {
        // CAN IDs (Phoenix Tuner X와 일치)
        public static final int LEFT_FRONT_ID  = 1;
        public static final int LEFT_REAR_ID   = 2;
        public static final int RIGHT_FRONT_ID = 3;
        public static final int RIGHT_REAR_ID  = 4;

        // 텔레옵 속도 스케일
        public static final double DRIVE_SPEED_SCALE    = 0.7;
        public static final double DRIVE_ROTATION_SCALE = 0.6;

        // 전류 제한 (A)
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

        // [가정] 물리 파라미터 — 실측 후 수정
        public static final double WHEEL_DIAMETER_METERS = 0.1524;   // 6인치 휠
        public static final double GEAR_RATIO             = 5.95;    // AM14U6 기본 감속비
        public static final double TRACK_WIDTH_METERS     = 0.561;   // ~24인치 (좌우 바퀴 중심 간격)

        // 엔코더 변환 계수: TalonFX 회전수(rot) → 이동 거리(m)
        public static final double ENCODER_POSITION_FACTOR =
            (Math.PI * WHEEL_DIAMETER_METERS) / GEAR_RATIO;

        // [가정] PathPlanner용 최대 속도 — 실측 권장
        public static final double MAX_SPEED_MPS = 5.45;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 0;
    }

    public static final class ShooterConstants {
        // CAN ID — Phoenix Tuner X에서 Kraken X60 설정과 반드시 일치
        public static final int SHOOTER_MOTOR_5_ID = 5;
        public static final int SHOOTER_MOTOR_6_ID = 6;
        public static final int SHOOTER_MOTOR_7_ID = 7;

        // 전류 제한 (A) — Kraken X60 연속 권장: 60~80A
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    }

    // =========================================================
    // 인테이크 상수
    // =========================================================
    public static final class IntakeConstants {

        // ── CAN ID ──
        /** 인테이크 롤러 (Kraken X60 / TalonFX) */
        public static final int INTAKE_ROLLER_ID = 8;
        /** 인테이크 피벗 (NEO 1.1 / SparkMax Brushless) */
        public static final int INTAKE_PIVOT_ID  = 9;

        // ── 롤러 설정 ──
        /** 롤러 출력 (0.0 ~ 1.0) */
        public static final double ROLLER_SPEED = 0.8;

        /**
         * ★ 롤러 방향 반전 플래그
         *   false = 버튼 누르면 정회전 (인테이크 OUT 방향)
         *   true  = 버튼 누르면 역회전 (인테이크 IN 방향)
         */
        public static final boolean ROLLER_INVERTED = true;

        /** 롤러 전류 제한 (A) — Kraken X60 연속 권장: 60~80A */
        public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

        // ── 피벗 설정 ──
        /** 피벗 출력 (0.0 ~ 1.0) */
        public static final double PIVOT_SPEED = 0.3;

        /**
         * NEO 1.1 스마트 전류 제한 (A)
         * 이 값을 초과하면 소프트웨어가 모터를 정지시킵니다 (기계적 한계 도달 감지용).
         * NEO 1.1 정격 연속: ~40A, 피벗 안전 제한: 20A
         */
        public static final double PIVOT_CURRENT_LIMIT_AMPS = 20.0;

        /**
         * 하드웨어 레벨 절대 전류 차단 (A)
         * SparkMax가 이 값을 초과하면 즉시 출력을 차단합니다.
         */
        public static final double PIVOT_SECONDARY_CURRENT_LIMIT_AMPS = 25.0;
    }

    // =========================================================
    // 컨베이어 상수
    // =========================================================
    public static final class ConveyorConstants {

        // ── CAN ID ──
        /** 컨베이어 벨트 (CIM / SparkMax Brushed) */
        public static final int CONVEYOR_MOTOR_ID = 10;

        /** 컨베이어 출력 (0.0 ~ 1.0) */
        public static final double CONVEYOR_SPEED = 0.6;

        /**
         * ★ 컨베이어 방향 반전 플래그
         *   false = 버튼 누르면 정회전 (슈터 방향으로 이송)
         *   true  = 버튼 누르면 역회전
         *   → 기구 조립 후 실제 방향 확인하여 변경하세요.
         */
        public static final boolean CONVEYOR_INVERTED = false;

        /** 컨베이어 전류 제한 (A) — CIM 연속 권장: 30~40A */
        public static final double CONVEYOR_CURRENT_LIMIT_AMPS = 35.0;
    }
}
