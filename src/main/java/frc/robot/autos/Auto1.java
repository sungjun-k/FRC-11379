package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * 오토 1번 (김도윤) - 슈터 + 컨베이어 동시 구동
 *
 * 순서:
 *   1. 슈터 스핀업 1초
 *   2. 슈터 + 컨베이어 동시 2초
 *   3. 모두 정지
 *
 * ★ ShooterCommand 파라미터는 기존 TeleOp 방식과 달리
 *   오토에서는 고정 출력(1.0)으로 발사합니다.
 */
public class Auto1 extends SequentialCommandGroup {

    private static final double SPINUP_SECONDS  = 1.0;
    private static final double SHOOT_SECONDS   = 2.0;

    public Auto1(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
        addCommands(
            // 슈터 스핀업
            new edu.wpi.first.wpilibj2.command.RunCommand(
                () -> shooter.setSpeed(1.0), shooter
            ).withTimeout(SPINUP_SECONDS),

            // 슈터 + 컨베이어 동시 발사
            new ParallelCommandGroup(
                new edu.wpi.first.wpilibj2.command.RunCommand(
                    () -> shooter.setSpeed(1.0), shooter
                ).withTimeout(SHOOT_SECONDS),
                new ConveyorCommand(conveyor).withTimeout(SHOOT_SECONDS)
            ),

            // 정지
            new edu.wpi.first.wpilibj2.command.InstantCommand(
                () -> shooter.setSpeed(0), shooter
            )
        );
    }
}
