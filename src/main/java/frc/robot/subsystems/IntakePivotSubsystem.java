package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
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
 * 인테이크 피벗 서브시스템
 *
 * ⚠️ PID는 m_pidEnabled == true 일 때만 실행됩니다.
 *    버튼을 누르는 동안만 IntakePivotCommand이 enablePID() 호캜 후
 *    목표를 설정하고, 버튼을 떼면 disablePID() 호캜합니다.
 *    따라서 텔레옵 EN 시잡로 인테이크가 올라가는 문제가 없습니다.
 */
public class IntakePivotSubsystem extends SubsystemBase {

    private final SparkMax                  m_pivotMotor;
    private final RelativeEncoder           m_encoder;
    private final SparkClosedLoopController m_controller;

    private double  m_targetPosition = IntakeConstants.PIVOT_HOME_POS;
    private boolean m_pidEnabled     = false; // 기본값: PID OFF

    public IntakePivotSubsystem() {
        m_pivotMotor = new SparkMax(IntakeConstants.INTAKE_PIVOT_ID, MotorType.kBrushless);

        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(IntakeConstants.PIVOT_FORWARD_SOFT_LIMIT)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(IntakeConstants.PIVOT_REVERSE_SOFT_LIMIT);

        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig
            .p(IntakeConstants.PIVOT_KP)
            .i(IntakeConstants.PIVOT_KI)
            .d(IntakeConstants.PIVOT_KD)
            .outputRange(-0.6, 0.6); // 지글 속도를 위해 기존 ±0.3 → ±0.6 상향

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit((int) IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS)
            .secondaryCurrentLimit(IntakeConstants.PIVOT_SECONDARY_CURRENT_LIMIT_AMPS)
            .idleMode(IdleMode.kBrake)
            .apply(softLimitConfig)
            .apply(closedLoopConfig);

        m_pivotMotor.configure(config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        m_encoder    = m_pivotMotor.getEncoder();
        m_controller = m_pivotMotor.getClosedLoopController();

        m_encoder.setPosition(IntakeConstants.PIVOT_HOME_POS);
    }

    // ── PID 활성화 / 비활성화 ──────────────────────────────────
    public void enablePID() {
        m_pidEnabled = true;
    }

    public void disablePID() {
        m_pidEnabled = false;
        m_pivotMotor.set(0);
    }

    // ── 목표 위치 설정 ──────────────────────────────────────
    public void deployIntake()  { m_targetPosition = IntakeConstants.PIVOT_INTAKE_POS; }
    public void retractIntake() { m_targetPosition = IntakeConstants.PIVOT_HOME_POS;   }
    public void holdCurrentPosition() { m_targetPosition = m_encoder.getPosition();    }

    /**
     * 임의 엔코더 위치를 목표로 설정 (IntakeJiggleCommand 등에서 사용)
     */
    public void setTarget(double position) {
        m_targetPosition = position;
    }

    public void bumpDown() {
        m_targetPosition = Math.min(
            m_targetPosition + IntakeConstants.PIVOT_BUMP_STEP,
            IntakeConstants.PIVOT_FORWARD_SOFT_LIMIT);
    }
    public void bumpUp() {
        m_targetPosition = Math.max(
            m_targetPosition - IntakeConstants.PIVOT_BUMP_STEP,
            IntakeConstants.PIVOT_REVERSE_SOFT_LIMIT);
    }

    // ── 상태 조회 ────────────────────────────────────────────
    public double  getPosition()    { return m_encoder.getPosition(); }
    public boolean isAtTarget()     { return Math.abs(m_encoder.getPosition() - m_targetPosition) < 0.3; }
    public boolean isOverCurrent()  { return m_pivotMotor.getOutputCurrent() > IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS; }
    public void    resetEncoder()   {
        m_encoder.setPosition(IntakeConstants.PIVOT_HOME_POS);
        m_targetPosition = IntakeConstants.PIVOT_HOME_POS;
    }

    // ── 중력 보상 FF ───────────────────────────────────────
    private double getGravityFF() {
        double deg = m_encoder.getPosition() * (360.0 / IntakeConstants.PIVOT_GEAR_RATIO);
        double angleFromHoriz = IntakeConstants.PIVOT_HOME_ANGLE_DEG - deg;
        return IntakeConstants.PIVOT_KG_VOLTS * Math.cos(Math.toRadians(angleFromHoriz));
    }

    // ── periodic ────────────────────────────────────────────
    @Override
    public void periodic() {
        if (m_pidEnabled) {
            m_controller.setReference(
                m_targetPosition,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                getGravityFF(),
                ArbFFUnits.kVoltage
            );
        }

        SmartDashboard.putNumber("Pivot/Position",  m_encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Target",    m_targetPosition);
        SmartDashboard.putNumber("Pivot/Current",   m_pivotMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Pivot/PID_ON",   m_pidEnabled);
        SmartDashboard.putBoolean("Pivot/AtTarget", isAtTarget());
    }
}
