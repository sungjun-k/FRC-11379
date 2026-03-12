package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * 인테이크 롤러 커맨드
 * reversed=false → 정방향 흡입 (RB)
 * reversed=true  → 역방향 토출 (LB)
 */
public class IntakeRollerCommand extends Command {

    private final IntakeRollerSubsystem m_intake;
    private final boolean m_reversed;

    /** 정방향 흡입 */
    public IntakeRollerCommand(IntakeRollerSubsystem intake) {
        this(intake, false);
    }

    /** reversed=true 이면 역방향 구동 */
    public IntakeRollerCommand(IntakeRollerSubsystem intake, boolean reversed) {
        m_intake = intake;
        m_reversed = reversed;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        double direction = IntakeConstants.ROLLER_INVERTED ? -1.0 : 1.0;
        if (m_reversed) direction = -direction;
        m_intake.setSpeed(direction * IntakeConstants.ROLLER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
