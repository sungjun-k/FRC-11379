package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ConveyorCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Auto 1 (김도윤) - 슈터 스핀업 → 슈터 + 컨베이어 동시 발사
 *
 * 순서:
 *   1. 슈터 스핀업 SPINUP_SECONDS초
 *   2. 슈터 + 컨베이어 동시 SHOOT_SECONDS초
 *   3. 정지
 */
public class Auto1 extends SequentialCommandGroup {

    private static final double SPINUP_SECONDS = 1.0;
    private static final double SHOOT_SECONDS  = 2.0;
    private static final double SHOOT_OUTPUT   = 1.0; // 0.0 ~ 1.0

    public Auto1(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
        addCommands(
            // 1. 슈터 스핀업 (컨베이어 대기)
            new RunCommand(() -> {
                shooter.setOutput(SHOOT_OUTPUT);
                shooter.setMotor6Fixed(SHOOT_OUTPUT);
            }, shooter).withTimeout(SPINUP_SECONDS),

            // 2. 슈터 + 컨베이어 동시 발사
            new ParallelCommandGroup(
                new RunCommand(() -> {
                    shooter.setOutput(SHOOT_OUTPUT);
                    shooter.setMotor6Fixed(SHOOT_OUTPUT);
                }, shooter).withTimeout(SHOOT_SECONDS),
                new ConveyorCommand(conveyor).withTimeout(SHOOT_SECONDS)
            ),

            // 3. 정지
            new InstantCommand(() -> shooter.stop(), shooter)
        );
    }
}
