package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

/**
 * 인테이크 피벗 커맨드
 *
 * 버튼 누르는 순간: enablePID() → 목표 위치 설정 → PID 실행
 * 버튼 뜨는 순간: disablePID() → 모터 set(0) → kBrake로 제자리 유지
 *
 * ⚠️ 이 시라스틸 덕분에 텔레옥 EN 시 인테이크가 제멘대로 올라가지 않습니다.
 */
public class IntakePivotCommand extends Command {

    public enum Direction { DOWN, UP, BUMP_DOWN, BUMP_UP }

    private final IntakePivotSubsystem m_pivot;
    private final Direction            m_direction;

    public IntakePivotCommand(IntakePivotSubsystem pivot, Direction direction) {
        m_pivot     = pivot;
        m_direction = direction;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        m_pivot.enablePID(); // 버튼 누르는 순간 PID ON
        if (m_direction == Direction.BUMP_DOWN) m_pivot.bumpDown();
        if (m_direction == Direction.BUMP_UP)   m_pivot.bumpUp();
    }

    @Override
    public void execute() {
        switch (m_direction) {
            case DOWN: m_pivot.deployIntake();  break;
            case UP:   m_pivot.retractIntake(); break;
            default:   break; // BUMP는 initialize()에서 이미 처리
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.disablePID(); // 버튼 뜨는 순간 PID OFF + 모터 정지
    }

    @Override
    public boolean isFinished() {
        // BUMP는 한 번만 실행
        return m_direction == Direction.BUMP_DOWN || m_direction == Direction.BUMP_UP;
    }
}
