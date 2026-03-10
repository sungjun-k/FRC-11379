package frc.robot;

public final class Constants {

    public static final class DriveConstants {
        // CAN IDs (Phoenix Tuner X와 일치)
        public static final int LEFT_FRONT_ID  = 1;
        public static final int LEFT_REAR_ID   = 2;
        public static final int RIGHT_FRONT_ID = 3;
        public static final int RIGHT_REAR_ID  = 4;

        // 텔레옵 속도 스케일
        public static final double DRIVE_SPEED_SCALE    = 0.6;
        public static final double DRIVE_ROTATION_SCALE = 0.35;

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
        public static final double MAX_SPEED_MPS = 5.0; // 5 m/s (약 18 km/h)
        
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
        public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    }

    // =========================================================
    // 인테이크 상수
    // =========================================================
    public static final class IntakeConstants {

        // ── CAN ID ──
        public static final int INTAKE_ROLLER_ID = 8;
        public static final int INTAKE_PIVOT_ID  = 9;

        // ── 롤러 설정 ──
        public static final double ROLLER_SPEED = 0.8;
        public static final boolean ROLLER_INVERTED = true;
        public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

        // ── 피벗 전류 제한 ──
        public static final double PIVOT_CURRENT_LIMIT_AMPS           = 25.0;
        public static final double PIVOT_SECONDARY_CURRENT_LIMIT_AMPS = 30.0;

        // ── 피벗 Position PID 게인 (SparkMax 온보드 PID) ──
        // 튜닝 방법: kP부터 올리며 진동 없는 최대값 찾기 → kD로 오버슈트 억제
        public static final double PIVOT_KP = 0.08;  // [가정] 진동 시 낮추기
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.005; // [가정] 오버슈트 시 올리기

        // ── 피벗 중력 보상 (Gravity Feedforward, 단위: Volt) ──
        // 의미: 피벗이 수평일 때 중력을 버티는 데 필요한 전압
        // 튜닝 방법: 수평 자세에서 버튼 떼고 처지면 올리기, 반대로 올라가면 낮추기
        public static final double PIVOT_KG_VOLTS = 0.5; // [가정]

        // ── 피벗 기하학 (중력 보상 각도 계산용) ──
        // PIVOT_HOME_ANGLE_DEG: 홈(수납) 위치에서 '수평 기준' 각도
        //   예: 인테이크가 완전히 수직으로 접히면 90, 약간 비스듬히 접히면 75
        public static final double PIVOT_HOME_ANGLE_DEG = 90.0; // [가정] 실측 필요
        // PIVOT_GEAR_RATIO: 모터 → 피벗 축의 감속비
        public static final double PIVOT_GEAR_RATIO = 80.0;     // [가정] 실측 필요

        // ── 피벗 목표 위치 (모터 회전수, 홈 = 0.0 기준) ──
        // 홈 = 0.0 (전원 켤 때 피벗이 반드시 이 위치에 있어야 함!)
        public static final double PIVOT_HOME_POS   = 0.0;
        // 인테이크 전개 위치: SmartDashboard "Pivot/Position (rot)" 확인 후 실측값으로 교체
        public static final double PIVOT_INTAKE_POS = -11; // [가정] 실측 필요
        // 한 번 이동량 (rotations)
        public static final double PIVOT_BUMP_STEP  = 0.3;

        // ── 소프트 리밋 (float, SparkMax 요구) ──
        // 절대 넘어가면 안 되는 하드웨어 경계 — SparkMax가 출력 차단
        public static final float PIVOT_FORWARD_SOFT_LIMIT = 1.0f; // 최대 아래 (INTAKE_POS + 여유)
        public static final float PIVOT_REVERSE_SOFT_LIMIT = -12f; // 최대 위 (HOME보다 약간 위 여유)
    }

    // =========================================================
    // 컨베이어 상수
    // =========================================================
    public static final class ConveyorConstants {

        public static final int CONVEYOR_MOTOR_ID = 10;
        public static final double CONVEYOR_SPEED = 1;
        public static final boolean CONVEYOR_INVERTED = false;
        public static final double CONVEYOR_CURRENT_LIMIT_AMPS = 35.0;
    }
}
