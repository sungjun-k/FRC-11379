package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * 오토 0번 (송주원) - 드라이브 전진 + 인테이크 동시 구동
 *
 * 순서:
 *   1. 드라이브 전진 2초 + 인테이크 동시 흡입
 *   2. 인테이크 정지 및 드라이브 정지
 *
 * ★ 전진 속도/시간은 현장에서 튜닝하세요.
 */
public class Auto0 extends SequentialCommandGroup {

    private static final double DRIVE_SPEED   = 0.5;  // 가정: 50% 출력
    private static final double DRIVE_SECONDS = 2.0;  // 가정: 2초 전진

    public Auto0(DriveSubsystem drive, IntakeRollerSubsystem intake) {
        addCommands(
            new ParallelCommandGroup(
                // 전진
                new edu.wpi.first.wpilibj2.command.RunCommand(
                    () -> drive.arcadeDrive(DRIVE_SPEED, 0), drive
                ).withTimeout(DRIVE_SECONDS),
                // 인테이크 동시 구동
                new IntakeRollerCommand(intake).withTimeout(DRIVE_SECONDS)
            ),
            // 정지
            new edu.wpi.first.wpilibj2.command.InstantCommand(
                () -> drive.arcadeDrive(0, 0), drive
            )
        );
    }
}
