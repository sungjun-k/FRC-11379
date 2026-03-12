package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * 슈터 커맨드
 *
 * 오퍼레이터 버튼 배치:
 *   RB 누르는 동안  → 정방향 발사 (1.0 고정)
 *   RT 누르는 동안  → 역력 구동 (RT 축 만큼)
 *   D-pad ↑       → 목표 속도 +5%
 *   D-pad ↓       → 목표 속도 -5%
 *
 * 두 버튼 모두 눥르면 RB 우선
 */
public class ShooterCommand extends Command {

    @FunctionalInterface
    public interface BooleanSupplier {
        boolean getAsBoolean();
    }

    @FunctionalInterface
    public interface DoubleSupplier {
        double getAsDouble();
    }

    private final ShooterSubsystem m_shooter;
    private final BooleanSupplier  m_shootForward; // RB
    private final DoubleSupplier   m_shootReverse; // RT axis
    private final BooleanSupplier  m_speedUp;      // D-pad UP
    private final BooleanSupplier  m_speedDown;    // D-pad DOWN

    private double m_targetSpeed = 1.0; // RB 누르면 이 값으로 발사

    public ShooterCommand(
        ShooterSubsystem shooter,
        BooleanSupplier shootForward,
        DoubleSupplier  shootReverse,
        BooleanSupplier speedUp,
        BooleanSupplier speedDown
    ) {
        m_shooter      = shooter;
        m_shootForward = shootForward;
        m_shootReverse = shootReverse;
        m_speedUp      = speedUp;
        m_speedDown    = speedDown;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        // D-pad 속도 조절
        if (m_speedUp.getAsBoolean()) {
            m_targetSpeed = Math.min(1.0, m_targetSpeed + 0.05);
        } else if (m_speedDown.getAsBoolean()) {
            m_targetSpeed = Math.max(0.0, m_targetSpeed - 0.05);
        }

        if (m_shootForward.getAsBoolean()) {
            // RB: 정방향 발사
            m_shooter.setOutput(m_targetSpeed);
            m_shooter.setMotor6Fixed(m_targetSpeed);
        } else {
            double rev = m_shootReverse.getAsDouble();
            if (rev > 0.05) {
                // RT: 역방향 (트리거 측랑만큼 출력)
                m_shooter.setOutput(-rev);
                m_shooter.setMotor6Fixed(-rev);
            } else {
                m_shooter.stop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
