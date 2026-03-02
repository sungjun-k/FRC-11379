package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_zRotation;

    // ── 직진 보정용 ──────────────────────────────
    private double m_targetAngle = 0.0;
    private static final double STRAIGHT_KP      = 0.02;
    private static final double TURN_DEADBAND     = 0.05;
    private static final double FORWARD_DEADBAND  = 0.1;

    // ── 가감속 제한 (SlewRateLimiter) ──────────────────
    /**
     * 전진/후진 가속 제한
     * 단위: 1초당 변화량 (units/s, 0.0~1.0 스케일 기준)
     * 
     * 너무 느리게 벘어짐 → 올리기 (3.0, 4.0)
     * 여전히 전복 위험 → 낮춰라 (1.0, 1.5)
     */
    private static final double FORWARD_SLEW_RATE  = 2.0;

    /**
     * 회전 가속 제한
     * 전진보다 약간 빠르게 설정해야 회전 응답성이 좋다
     */
    private static final double ROTATION_SLEW_RATE = 2.5;

    private final SlewRateLimiter m_forwardLimiter  = new SlewRateLimiter(FORWARD_SLEW_RATE);
    private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(ROTATION_SLEW_RATE);

    public DriveCommand(
        DriveSubsystem drive,
        DoubleSupplier xSpeed,
        DoubleSupplier zRotation
    ) {
        m_drive     = drive;
        m_xSpeed    = xSpeed;
        m_zRotation = zRotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        // ① 가감속 적용: 조이스틱 값을 바로 쓰지 않고 Limiter를 거쳐 부드럽게 전달
        double forward = m_forwardLimiter.calculate(m_xSpeed.getAsDouble());
        double turn    = m_zRotation.getAsDouble(); // 회전은 직진 보정에서 다시 처리

        // ② 직진 보정: 조이스틱 회전 입력이 없을 때 자이로로 각도 유지
        if (Math.abs(turn) < TURN_DEADBAND && Math.abs(forward) > FORWARD_DEADBAND) {
            double headingError = m_targetAngle - m_drive.getGyroAngle();
            turn = headingError * STRAIGHT_KP;
        } else {
            m_targetAngle = m_drive.getGyroAngle();
        }

        // ③ 회전에도 가감속 적용 (직진 보정 개입 후)
        turn = m_rotationLimiter.calculate(turn);

        m_drive.arcadeDrive(forward, turn);
    }

    @Override
    public void end(boolean interrupted) {
        // 커맨드 종료 시 Limiter도 0으로 리셋
        m_forwardLimiter.reset(0);
        m_rotationLimiter.reset(0);
        m_drive.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
