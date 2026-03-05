package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * 인테이크 피벗 서브시스템 (Position PID + Soft Limit)
 *
 * 동작 원리:
 *   - 항상 m_targetPosition을 향해 Position PID로 이동
 *   - Soft Limit: FORWARD/REVERSE 범위 이탈 시 SparkMax가 하드웨어 차단
 *   - 중력 보상(ArbFF)은 setReference의 arbFF 파라미터가 REVLib 2025+에서 deprecated되어
 *     현재 SmartDashboard 모니터링만 수행함. 필요 시 피드포워드 대체 구현 필요.
 *
 * 중요: 전원 켤 때 피벗이 반드시 홈(수납) 위치에 있어야 소프트 리미트가 정확함!
 */
public class IntakePivotSubsystem extends SubsystemBase {

    private final SparkMax                  m_pivotMotor;
    private final RelativeEncoder           m_encoder;
    private final SparkClosedLoopController m_controller;

    // 현재 PID 목표 위치 (rotations). 항상 이 값을 향해 PID가 동작함
    private double m_targetPosition = IntakeConstants.PIVOT_HOME_POS;

    public IntakePivotSubsystem() {
        m_pivotMotor = new SparkMax(IntakeConstants.INTAKE_PIVOT_ID, MotorType.kBrushless);

        // ── Soft Limit 설정 ──
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(IntakeConstants.PIVOT_FORWARD_SOFT_LIMIT)  // 최대 위 한계
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(IntakeConstants.PIVOT_REVERSE_SOFT_LIMIT); // 최대 아래 한계

        // ── Position PID 설정 (SparkMax 온보드) ──
        // feedbackSensor 생략 → SparkMax는 기본값이 이미 kPrimaryEncoder
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig
            .p(IntakeConstants.PIVOT_KP)
            .i(IntakeConstants.PIVOT_KI)
            .d(IntakeConstants.PIVOT_KD)
            .outputRange(-0.2, 0.2); // 최대 출력 20% 제한: 급격한 동작 방지

        // ── 전체 SparkMax 설정 ──
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit((int) IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS)
            .secondaryCurrentLimit(IntakeConstants.PIVOT_SECONDARY_CURRENT_LIMIT_AMPS)
            .idleMode(IdleMode.kBrake)
            .apply(softLimitConfig)
            .apply(closedLoopConfig);

        m_pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder    = m_pivotMotor.getEncoder();
        m_controller = m_pivotMotor.getClosedLoopController();

        // 전원 켤 때 현재 위치를 0(홈)으로 초기화
        m_encoder.setPosition(IntakeConstants.PIVOT_HOME_POS);
        m_targetPosition = IntakeConstants.PIVOT_HOME_POS;
    }

    // ───────────────────────────────────────────────────────────
    // ★ 중력 보상 값 계산 (SmartDashboard 모니터링 전용)
    // 공식: kG * cos(수평 기준 현재 각도)
    //   수평(0°)에서 cos=1.0 → 최대 보상  (중력이 가장 센 자세)
    //   수직 위(90°)에서 cos=0.0 → 보상 없음 (중력 토크 없음)
    // ───────────────────────────────────────────────────────────
    private double getGravityFeedforward() {
        double degreesRotated = m_encoder.getPosition() * (360.0 / IntakeConstants.PIVOT_GEAR_RATIO);
        double angleFromHorizontalDeg = IntakeConstants.PIVOT_HOME_ANGLE_DEG - degreesRotated;
        return IntakeConstants.PIVOT_KG_VOLTS * Math.cos(Math.toRadians(angleFromHorizontalDeg));
    }

    // ───────────────────────────────────────────────────────────
    // 공개 제어 메서드
    // ───────────────────────────────────────────────────────────

    /** 인테이크 전개 위치로 이동 (버튼 누르는 동안 호출) */
    public void deployIntake() {
        m_targetPosition = IntakeConstants.PIVOT_INTAKE_POS;
    }

    /** 홈(수납) 위치로 이동 */
    public void retractIntake() {
        m_targetPosition = IntakeConstants.PIVOT_HOME_POS;
    }

    /**
     * 현재 위치에서 그대로 멈춴 (버튼 돼면 호출)
     * kBrake 모드 + PID로 같은 위치를 유지
     */
    public void holdCurrentPosition() {
        m_targetPosition = m_encoder.getPosition();
    }

    /** 아래 (BUMP_STEP만큼 목표를 더 내림) */
    public void bumpDown() {
        double newTarget = m_targetPosition + IntakeConstants.PIVOT_BUMP_STEP;
        m_targetPosition = Math.min(newTarget, IntakeConstants.PIVOT_FORWARD_SOFT_LIMIT);
    }

    /** 위 (BUMP_STEP만큼 목표를 더 올림) */
    public void bumpUp() {
        double newTarget = m_targetPosition - IntakeConstants.PIVOT_BUMP_STEP;
        m_targetPosition = Math.max(newTarget, IntakeConstants.PIVOT_REVERSE_SOFT_LIMIT);
    }

    /** 현재 엔코더 위치 반환 (rotations) */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /** 목표 위치 도달 여부 */
    public boolean isAtTarget() {
        return Math.abs(m_encoder.getPosition() - m_targetPosition) < 0.3;
    }

    /** 과전류 여부 */
    public boolean isOverCurrent() {
        return m_pivotMotor.getOutputCurrent() > IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS;
    }

    /** 엔코더 0점 강제 초기화 */
    public void resetEncoder() {
        m_encoder.setPosition(IntakeConstants.PIVOT_HOME_POS);
        m_targetPosition = IntakeConstants.PIVOT_HOME_POS;
    }

    // ───────────────────────────────────────────────────────────
    // 주기적 실행: Position PID로 m_targetPosition 유지
    // ───────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        // SparkMax 온보드 Position PID (3파라미터, REVLib 2025+ deprecated API 미사용)
        m_controller.setReference(
            m_targetPosition,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );

        SmartDashboard.putNumber("Pivot/Position (rot)", m_encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Target (rot)",   m_targetPosition);
        SmartDashboard.putNumber("Pivot/Current (A)",    m_pivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Pivot/GravityFF (V)",  getGravityFeedforward()); // 모니터링 전용
        SmartDashboard.putBoolean("Pivot/AtTarget",      isAtTarget());
    }
}
