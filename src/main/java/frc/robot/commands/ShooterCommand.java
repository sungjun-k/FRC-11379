package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * 슈터 커맨드 (DefaultCommand)
 *
 * 오퍼레이터 버튼:
 *   RB            → 정방향 발사 (m_targetSpeed)
 *   RT            → 역방향 (RT 축량 비례)
 *   D-pad ↑ / ↓  → 발사 속도 ±5%
 */
public class ShooterCommand extends Command {

    private final ShooterSubsystem m_shooter;
    private final BooleanSupplier  m_shootForward;  // RB
    private final DoubleSupplier   m_shootReverse;  // RT axis
    private final BooleanSupplier  m_speedUp;       // D-pad UP
    private final BooleanSupplier  m_speedDown;     // D-pad DOWN

    private double m_targetSpeed = 1.0;

    public ShooterCommand(
        ShooterSubsystem shooter,
        BooleanSupplier  shootForward,
        DoubleSupplier   shootReverse,
        BooleanSupplier  speedUp,
        BooleanSupplier  speedDown
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
        if (m_speedUp.getAsBoolean()) {
            m_targetSpeed = Math.min(1.0, m_targetSpeed + 0.05);
        } else if (m_speedDown.getAsBoolean()) {
            m_targetSpeed = Math.max(0.0, m_targetSpeed - 0.05);
        }

        if (m_shootForward.getAsBoolean()) {
            m_shooter.setOutput(m_targetSpeed);
            m_shooter.setMotor6Fixed(m_targetSpeed);
        } else {
            double rev = m_shootReverse.getAsDouble();
            if (rev > 0.05) {
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
