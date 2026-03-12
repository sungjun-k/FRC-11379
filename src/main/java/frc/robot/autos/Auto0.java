package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * Auto 0 (송주원) - 드라이브 전진 + 인테이크 동시 흡입
 *
 * 튜닝 포인트:
 *   DRIVE_SPEED   - 전진 출력 (0.0 ~ 1.0)
 *   DRIVE_SECONDS - 전진 시간 (초)
 */
public class Auto0 extends SequentialCommandGroup {

    private static final double DRIVE_SPEED   = 0.5;
    private static final double DRIVE_SECONDS = 2.0;

    public Auto0(DriveSubsystem drive, IntakeRollerSubsystem intake) {
        addCommands(
            new ParallelCommandGroup(
                new RunCommand(
                    () -> drive.arcadeDrive(DRIVE_SPEED, 0), drive
                ).withTimeout(DRIVE_SECONDS),
                new IntakeRollerCommand(intake).withTimeout(DRIVE_SECONDS)
            ),
            new InstantCommand(() -> drive.arcadeDrive(0, 0), drive)
        );
    }
}
