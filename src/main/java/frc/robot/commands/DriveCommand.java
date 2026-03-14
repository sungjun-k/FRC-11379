package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_zRotation;

    // ── 가감속 제한 (SlewRateLimiter) ──────────────────
    /**
     * 전진/후진 가속 제한
     * 단위: 1초당 변화량 (units/s, 0.0~1.0 스케일 기준)
     *
     * 너무 느리게 밟아짐 → 올리기 (3.0, 4.0)
     * 여전히 전복 위험 → 낮추기 (1.0, 1.5)
     */
    private static final double FORWARD_SLEW_RATE  = 3.0;

    /**
     * 회전 가속 제한
     * 전진보다 약간 빠르게 설정해야 회전 응답성이 좋다
     */
    private static final double ROTATION_SLEW_RATE = 4.0;

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
        double forward = m_forwardLimiter.calculate(m_xSpeed.getAsDouble());
        double turn    = m_rotationLimiter.calculate(m_zRotation.getAsDouble());

        m_drive.arcadeDrive(forward, turn);
    }

    @Override
    public void end(boolean interrupted) {
        m_forwardLimiter.reset(0);
        m_rotationLimiter.reset(0);
        m_drive.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
