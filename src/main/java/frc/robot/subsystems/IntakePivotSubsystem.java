package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * 인테이크 피벗(각도 조절) 서브시스템
 * 모터: NEO 1.1 (Brushless) → SparkMax, CAN ID 9
 * - 20A 초과 시 소프트웨어 자동 정지 (기계적 한계 도달 감지)
 * - 내장 홀 이펙트 엔코더 사용 (상대 엔코더 → 켤 때 수납 위치에서 시작 필요)
 */
public class IntakePivotSubsystem extends SubsystemBase {

    private final SparkMax m_pivotMotor;
    private final RelativeEncoder m_encoder;

    public IntakePivotSubsystem() {
        m_pivotMotor = new SparkMax(IntakeConstants.INTAKE_PIVOT_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        // NEO 1.1 연속 전류 제한 (SmartCurrentLimit)
        config.smartCurrentLimit((int) IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS);
        // 하드웨어 레벨 절대 차단 (25A 이상이면 SparkMax가 출력 즉시 차단)
        config.secondaryCurrentLimit(IntakeConstants.PIVOT_SECONDARY_CURRENT_LIMIT_AMPS);
        // 브레이크 모드: 버튼을 뗐을 때 피벗이 중력으로 처지는 것을 방지
        config.idleMode(IdleMode.kBrake);

        m_pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_pivotMotor.getEncoder();
        // 전원 켤 때 반드시 수납(가장 위) 위치에서 시작 → 이 위치를 0점으로 초기화
        m_encoder.setPosition(0.0);
    }

    /**
     * 피벗을 위로 올립니다 (수납 방향).
     * 과전류 감지 시 즉시 정지합니다.
     */
    public void pivotUp() {
        if (isOverCurrent()) {
            stop();
            return;
        }
        m_pivotMotor.set(-IntakeConstants.PIVOT_SPEED);
    }

    /**
     * 피벗을 아래로 내립니다 (전개/인테이크 방향).
     * 과전류 감지 시 즉시 정지합니다.
     */
    public void pivotDown() {
        if (isOverCurrent()) {
            stop();
            return;
        }
        m_pivotMotor.set(IntakeConstants.PIVOT_SPEED);
    }

    /** 피벗 정지 */
    public void stop() {
        m_pivotMotor.set(0.0);
    }

    /**
     * 현재 엔코더 위치 반환 (단위: motor rotations)
     * 수납 위치 = 0.0, 전개 방향으로 내릴수록 음수 증가
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * 엔코더 0점 강제 초기화
     * 수납 위치에 도달했을 때 호출하여 기준점을 재설정할 수 있습니다.
     */
    public void resetEncoder() {
        m_encoder.setPosition(0.0);
    }

    /**
     * 과전류 여부 반환
     * PIVOT_CURRENT_LIMIT_AMPS(20A) 초과 시 true
     */
    public boolean isOverCurrent() {
        return m_pivotMotor.getOutputCurrent() > IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS;
    }

    @Override
    public void periodic() {
        // 과전류 감지 시 추가 안전망: 어느 경로로든 모터가 돌고 있으면 정지
        if (isOverCurrent()) {
            stop();
        }
        SmartDashboard.putNumber("Pivot/Position (rot)", getPosition());
        SmartDashboard.putNumber("Pivot/Current (A)", m_pivotMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Pivot/OverCurrent", isOverCurrent());
    }
}
