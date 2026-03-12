package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * RT 트리거를 누르는 동안 AprilTag ID 10 또는 26을 보고
 * 방향(Yaw)만 정렬하는 커맨드 (전진/후진 없음).
 *
 * 튜닝:
 *   P_GAIN        진동하면 낮춰라 (0.01), 느리면 올려라 (0.05)
 *   TOLERANCE_DEG 정지 정밀도 (1.0 ~ 3.0)
 */
public class AutoAlignCommand extends Command {

    private static final int    TARGET_ID_A    = 10;
    private static final int    TARGET_ID_B    = 26;
    private static final double P_GAIN         = 0.03;
    private static final double TOLERANCE_DEG  = 1.5;

    private final DriveSubsystem  m_drive;
    private final VisionSubsystem m_vision;

    public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
        m_drive  = drive;
        m_vision = vision;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        double yaw = m_vision.getTargetYawById(TARGET_ID_A, TARGET_ID_B);
        if (!Double.isNaN(yaw)) {
            // Yaw 양수(왼쪽) → 시계방향 회전 필요 → 음수 출력
            m_drive.arcadeDrive(0, -yaw * P_GAIN);
        } else {
            m_drive.arcadeDrive(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        double yaw = m_vision.getTargetYawById(TARGET_ID_A, TARGET_ID_B);
        return !Double.isNaN(yaw) && Math.abs(yaw) < TOLERANCE_DEG;
    }
}
