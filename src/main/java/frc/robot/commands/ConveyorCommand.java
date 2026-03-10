package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * 컨베이어 커맨드
 * 버튼을 누르는 동안 0.15초 돌고 0.4초 멈추는 패턴을 반복합니다.
 *
 * ★ 방향 반전이 필요하면:
 *   Constants.ConveyorConstants.CONVEYOR_INVERTED 를 true 로 변경하세요.
 */
public class ConveyorCommand extends Command {

    private final ConveyorSubsystem m_conveyor;
    private final Timer m_timer;
    private static final double RUN_TIME = 0.15;      // 0.15초 회전
    private static final double STOP_TIME = 0.4;      // 0.4초 정지
    private static final double CYCLE_TIME = RUN_TIME + STOP_TIME;  // 총 주기

    public ConveyorCommand(ConveyorSubsystem conveyor) {
        m_conveyor = conveyor;
        m_timer = new Timer();
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        // 주기 내에서의 경과 시간
        double cycleTime = m_timer.get() % CYCLE_TIME;

        if (cycleTime < RUN_TIME) {
            // 0.15초 동안 회전
            double direction = ConveyorConstants.CONVEYOR_INVERTED ? -1.0 : 1.0;
            m_conveyor.setSpeed(direction * ConveyorConstants.CONVEYOR_SPEED);
        } else {
            // 0.4초 동안 정지
            m_conveyor.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_conveyor.stop();
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
