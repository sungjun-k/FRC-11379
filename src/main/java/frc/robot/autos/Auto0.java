package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.IntakePivotCommand.Direction;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Auto 0 - Full routine
 *
 * 1. 인테이크 피벗 DOWN (전개)  1.5초
 * 2. 후진  DRIVE_SECONDS초
 * 3. 정지
 * 4. 슈터 스핀업  SPINUP_SECONDS초
 * 5. 슈터 유지 + 0.5초 뒤 컨베이어 ON -> 오토 시간까지 유지
 *
 * 튜닝 포인트:
 *   DRIVE_SPEED    후진 출력 (음수 = 후진)
 *   DRIVE_SECONDS  후진 시간 (초)
 *   SPINUP_SECONDS 슈터 스핀업 대기 (초)
 *   SHOOT_OUTPUT   슈터 출력 (0.0 ~ 1.0)
 *   SHOOT_SECONDS  슈터+컨베이어 발사 유지 시간 (초)
 */
public class Auto0 extends SequentialCommandGroup {

    private static final double DRIVE_SPEED    = -0.5; // 후진 = 음수
    private static final double DRIVE_SECONDS  = 2.0;
    private static final double SPINUP_SECONDS = 0.5;
    private static final double SHOOT_OUTPUT   = 1.0;
    private static final double SHOOT_SECONDS  = 10.0; // 남은 오토 시간 채우기

    public Auto0(
        DriveSubsystem drive,
        IntakePivotSubsystem pivot,
        ShooterSubsystem shooter,
        ConveyorSubsystem conveyor
    ) {
        addCommands(
            // 1. 인테이크 피벗 DOWN (전개)
            new IntakePivotCommand(pivot, Direction.DOWN)
                .withTimeout(1.5),

            // 2. 후진
            new RunCommand(() -> drive.arcadeDrive(DRIVE_SPEED, 0), drive)
                .withTimeout(DRIVE_SECONDS),

            // 3. 정지
            new InstantCommand(() -> drive.arcadeDrive(0, 0), drive),

            // 4. 슈터 스핀업
            new RunCommand(() -> {
                shooter.setOutput(SHOOT_OUTPUT);
                shooter.setMotor6Fixed(SHOOT_OUTPUT);
            }, shooter).withTimeout(SPINUP_SECONDS),

            // 5. 슈터 유지 + 0.5초 뒤 컨베이어 ON
            new ParallelCommandGroup(
                // 슈터는 SHOOT_SECONDS 내내 구동
                new RunCommand(() -> {
                    shooter.setOutput(SHOOT_OUTPUT);
                    shooter.setMotor6Fixed(SHOOT_OUTPUT);
                }, shooter).withTimeout(SHOOT_SECONDS),

                // 컨베이어는 0.5초 후 시작
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new ConveyorCommand(conveyor).withTimeout(SHOOT_SECONDS - 0.5)
                )
            ),

            // 6. 모두 정지
            new InstantCommand(() -> shooter.stop(), shooter)
        );
    }
}
