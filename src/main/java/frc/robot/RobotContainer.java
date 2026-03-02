package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.IntakePivotCommand.Direction;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    // ── 서브시스템 ────────────────────────────────────────────
    // ★ VisionSubsystem을 DriveSubsystem보다 먼저 선언해야 합니다.
    //   DriveSubsystem 생성자에 m_vision을 주입하는 구조이기 때문입니다.
    private final VisionSubsystem       m_vision        = new VisionSubsystem();
    private final DriveSubsystem        m_drive         = new DriveSubsystem(m_vision);
    private final ShooterSubsystem      m_shooter       = new ShooterSubsystem();
    private final IntakeRollerSubsystem m_intakeRoller  = new IntakeRollerSubsystem();
    private final IntakePivotSubsystem  m_intakePivot   = new IntakePivotSubsystem();
    private final ConveyorSubsystem     m_conveyor      = new ConveyorSubsystem();

    // ── 컨트롤러 ──────────────────────────────────────────
    /** 포트 0: 드라이버 (드라이브 + 슈터 + 자동 정렬) */
    private final CommandXboxController m_driverController =
        new CommandXboxController(DRIVER_CONTROLLER_PORT);

    /**
     * 오퍼레이터 (인테이크 / 피벗 / 콘베이어)
     * 콘트롤러 한 대로 테스트하려면: OIConstants.OPERATOR_CONTROLLER_PORT = 0 유지
     *
     * 버튼 배치:
     *   A 버튼  (button 1) → 인테이크 롤러 구동
     *   LB 버튼 (button 5) → 피벗 UP
     *   RB 버튼 (button 6) → 피벗 DOWN
     *   B 버튼  (button 2) → 콘베이어 구동
     *   X 버튼  (button 3) → 제자리 AprilTag 조준
     */
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    private SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        configureBindings();

        try {
            m_autoChooser = AutoBuilder.buildAutoChooser();
        } catch (RuntimeException e) {
            DriverStation.reportError(
                "AutoBuilder not configured, using empty auto chooser",
                e.getStackTrace()
            );
            m_autoChooser = new SendableChooser<>();
        }

        SmartDashboard.putData("Auto Mode", m_autoChooser);
    }

    private void configureBindings() {

        // ── 드라이브: 직진 보정 활성화 ────────────────────────
        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getRightX()
            )
        );

        // ── 슈터: RT = 정방향, LT = 역방향 ────────────────────────
        m_shooter.setDefaultCommand(
            new ShooterCommand(
                m_shooter,
                m_driverController::getRightTriggerAxis,
                m_driverController::getLeftTriggerAxis
            )
        );

        // ── 자동 정렬: X 버튼 누르고 있는 동안 제자리 AprilTag 조준 ──
        m_driverController.x().whileTrue(new AutoAlignCommand(m_drive, m_vision));

        // ── 인테이크 롤러: A 버튼 누르는 동안 구동 ───────────────
        m_operatorController.a().whileTrue(new IntakeRollerCommand(m_intakeRoller));

        // ── 인테이크 피벗: LB = UP / RB = DOWN ─────────────────
        m_operatorController.leftBumper().whileTrue(
            new IntakePivotCommand(m_intakePivot, Direction.UP)
        );
        m_operatorController.rightBumper().whileTrue(
            new IntakePivotCommand(m_intakePivot, Direction.DOWN)
        );

        // ── 콘베이어: B 버튼 누르는 동안 구동 ────────────────
        m_operatorController.b().whileTrue(new ConveyorCommand(m_conveyor));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void resetGyro() {
        m_drive.zeroGyro();
    }
}
