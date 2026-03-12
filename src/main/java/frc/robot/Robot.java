package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TagDistanceHoldCommand;

public class Robot extends TimedRobot {

    private RobotContainer         m_robotContainer;
    private Command                m_autonomousCommand;
    private TagDistanceHoldCommand m_distHoldCommand;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // ── 오토 ──────────────────────────────────────────────────
    @Override
    public void autonomousInit() {
        m_robotContainer.resetGyro();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    // ── 텔레옥 ────────────────────────────────────────────────
    @Override
    public void teleopInit() {
        // 오토 커맨드 중단 (오토 뒤로 텔레옥 장시 실행 방지)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // 피벗 PID OFF 보장 (오토에서 내려간 체 텔레옥 시작될 때 올라가는 문제 방지)
        m_robotContainer.pivotDisable();
    }

    // ── 테스트 ────────────────────────────────────────────────
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        m_distHoldCommand = m_robotContainer.createDistHoldCommand();
        m_distHoldCommand.schedule();
    }

    @Override
    public void testExit() {
        if (m_distHoldCommand != null) {
            m_distHoldCommand.cancel();
        }
    }

    @Override public void disabledInit()       {}
    @Override public void disabledPeriodic()   {}
    @Override public void autonomousPeriodic() {}
    @Override public void teleopPeriodic()     {}
    @Override public void testPeriodic()       {}
}
