package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * 컨베이어 커맨드 - 버튼 누르는 동안 지속 구동 (패턴 제거)
 *
 * ★ 방향 반전: Constants.ConveyorConstants.CONVEYOR_INVERTED = true
 */
public class ConveyorCommand extends Command {

    private final ConveyorSubsystem m_conveyor;

    public ConveyorCommand(ConveyorSubsystem conveyor) {
        m_conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        double direction = ConveyorConstants.CONVEYOR_INVERTED ? -1.0 : 1.0;
        m_conveyor.setSpeed(direction * ConveyorConstants.CONVEYOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
