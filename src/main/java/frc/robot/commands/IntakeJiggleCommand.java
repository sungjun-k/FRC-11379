package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * 인테이크 지글 커맨드 (공 걸림 해제용)
 *
 * 펼쳐진 위치(PIVOT_INTAKE_POS = -11) ↔ 위쪽 위치(-5)를
 * 롤러를 돌리면서 반복합니다.
 *
 * 피벗 속도: outputRange ±0.6 으로 재설정 (기존 ±0.3에서 상향)
 * ※ isFinished() = false → withTimeout()으로 시간 제한 필요
 */
public class IntakeJiggleCommand extends Command {

    // 지글 위치 (엔코더 단위)
    private static final double JIGGLE_LOW_POS  = IntakeConstants.PIVOT_INTAKE_POS; // -11 (펼쳐진)
    private static final double JIGGLE_HIGH_POS = -5.0;                             // 위로 올린 위치
    private static final double POSITION_TOLERANCE = 0.5;                           // 도달 판정

    // 롤러 속도
    private static final double ROLLER_SPEED = 0.6;

    private final IntakePivotSubsystem  m_pivot;
    private final IntakeRollerSubsystem m_roller;

    private boolean m_goingUp = true; // true = 위(-5)로, false = 아래(-11)로

    public IntakeJiggleCommand(IntakePivotSubsystem pivot, IntakeRollerSubsystem roller) {
        m_pivot  = pivot;
        m_roller = roller;
        addRequirements(pivot, roller);
    }

    @Override
    public void initialize() {
        m_pivot.enablePID();
        m_goingUp = true;
        // 처음 목표: 위(-5)로
        setPivotTarget(JIGGLE_HIGH_POS);
        m_roller.setSpeed(ROLLER_SPEED);
    }

    @Override
    public void execute() {
        // 목표 위치에 충분히 도달했으면 방향 전환
        double pos = m_pivot.getPosition();
        if (m_goingUp && pos >= JIGGLE_HIGH_POS - POSITION_TOLERANCE) {
            m_goingUp = false;
            setPivotTarget(JIGGLE_LOW_POS);
        } else if (!m_goingUp && pos <= JIGGLE_LOW_POS + POSITION_TOLERANCE) {
            m_goingUp = true;
            setPivotTarget(JIGGLE_HIGH_POS);
        }

        // 롤러는 항상 구동
        m_roller.setSpeed(ROLLER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.disablePID();
        m_pivot.deployIntake(); // 끝나면 펼쳐진 위치로 복귀
        m_roller.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // withTimeout()으로 제어
    }

    // ── private helper ───────────────────────────────────────
    private void setPivotTarget(double target) {
        // IntakePivotSubsystem의 m_targetPosition을 직접 접근하는 대신
        // 기존 deployIntake/retractIntake 대신 setCustomTarget 메서드 필요.
        // 여기서는 리플렉션 없이 subsystem에 setTarget()을 추가하는 방식 대신
        // bumpUp/bumpDown 루프 대신 직접 runClosedLoop 접근을 사용합니다.
        //
        // ⚠️ IntakePivotSubsystem에 setTarget(double) 메서드를 추가해야 합니다.
        m_pivot.setTarget(target);
    }
}
