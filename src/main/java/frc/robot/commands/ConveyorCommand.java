package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * 컨베이어 커맨드
 * 버튼 누르는 동안 0.15초 ON → 0.40초 OFF 패턴 반복
 */
public class ConveyorCommand extends Command {

    private final ConveyorSubsystem m_conveyor;
    private final Timer m_timer = new Timer();

    private static final double CYCLE = ConveyorConstants.CONVEYOR_RUN_TIME
                                      + ConveyorConstants.CONVEYOR_STOP_TIME;

    public ConveyorCommand(ConveyorSubsystem conveyor) {
        m_conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        double t = m_timer.get() % CYCLE;
        if (t < ConveyorConstants.CONVEYOR_RUN_TIME) {
            double dir = ConveyorConstants.CONVEYOR_INVERTED ? -1.0 : 1.0;
            m_conveyor.setSpeed(dir * ConveyorConstants.CONVEYOR_SPEED);
        } else {
            m_conveyor.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_conveyor.stop();
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
