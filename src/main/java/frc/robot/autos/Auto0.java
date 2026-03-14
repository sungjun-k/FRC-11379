package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.IntakeJiggleCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.IntakePivotCommand.Direction;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Auto 0 - Full routine
 *
 * 1. 후진  DRIVE_SECONDS초
 * 2. 정지
 * 3. 인테이크 피벗 DOWN (전개)  1.5초
 * 4. [동시] 슈터 스핀업 + 컨베이어(0.5초 딜레이) + 지글(인테이크 왕복+롤러)
 *    - 슈터: 처음부터 SHOOT_SECONDS 내내 구동
 *    - 컨베이어: 0.5초 뒤 시작
 *    - 지글: JIGGLE_SECONDS 동안 피벗 -11↔-5 왕복 + 롤러 0.6
 * 5. 모두 정지
 */
public class Auto0 extends SequentialCommandGroup {

    private static final double JIGGLE_SECONDS = 10.0;
    private static final double DRIVE_SPEED    = -0.5;
    private static final double DRIVE_SECONDS  = 3.0;
    private static final double SPINUP_SECONDS = 0.5;
    private static final double SHOOT_OUTPUT   = 1.0;
    private static final double SHOOT_SECONDS  = 10.0;

    public Auto0(
        DriveSubsystem drive,
        IntakePivotSubsystem pivot,
        IntakeRollerSubsystem roller,
        ShooterSubsystem shooter,
        ConveyorSubsystem conveyor
    ) {
        addCommands(
            // 1. 후진
            new RunCommand(() -> drive.arcadeDrive(DRIVE_SPEED, 0), drive)
                .withTimeout(DRIVE_SECONDS),

            new InstantCommand(() -> drive.arcadeDrive(0, 0), drive),

            // 2. 인테이크 피벗 DOWN (전개)
            new IntakePivotCommand(pivot, Direction.DOWN)
                .withTimeout(1.5),

            // 3. 슈터 스핀업 (단독 선행)
            new RunCommand(() -> shooter.setOutput(SHOOT_OUTPUT), shooter)
                .withTimeout(SPINUP_SECONDS),

            // 4. [동시] 슈터 + 컨베이어 + 지글
            new ParallelCommandGroup(
                // 슈터: SHOOT_SECONDS 내내
                new RunCommand(() -> shooter.setOutput(SHOOT_OUTPUT), shooter)
                    .withTimeout(SHOOT_SECONDS),

                // 컨베이어: 0.5초 딜레이 후 시작
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new ConveyorCommand(conveyor).withTimeout(SHOOT_SECONDS - 0.5)
                ),

                // 지글: JIGGLE_SECONDS 동안 피벗 왕복 + 롤러
                new IntakeJiggleCommand(pivot, roller)
                    .withTimeout(JIGGLE_SECONDS)
            ),

            // 5. 모두 정지
            new InstantCommand(() -> shooter.stop(), shooter)
        );
    }
}
