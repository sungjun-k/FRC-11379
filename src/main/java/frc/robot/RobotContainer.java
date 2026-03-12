package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.Auto0;
import frc.robot.autos.Auto1;
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
    private final VisionSubsystem       m_vision        = new VisionSubsystem();
    private final DriveSubsystem        m_drive         = new DriveSubsystem(m_vision);
    private final ShooterSubsystem      m_shooter       = new ShooterSubsystem();
    private final IntakeRollerSubsystem m_intakeRoller  = new IntakeRollerSubsystem();
    private final IntakePivotSubsystem  m_intakePivot   = new IntakePivotSubsystem();
    private final ConveyorSubsystem     m_conveyor      = new ConveyorSubsystem();

    // ── 컨트롤러 ─────────────────────────────────────────────
    /**
     * 포트 0: 드라이버 (송주원)
     *   왼쪽 스틱 Y / 오른쪽 스틱 X → 탱크 드라이브
     *   RT  → AutoAlignCommand (AprilTag ID 10 or 26, 방향만 정렬)
     *   LB  → 인테이크 롤러 정방향 (흡입)
     *   RB  → 인테이크 롤러 역방향 (토출)
     *   Y   → 피벗 UP   A → 피벗 DOWN
     */
    private final CommandXboxController m_driverController =
        new CommandXboxController(DRIVER_CONTROLLER_PORT);

    /**
     * 포트 1: 오퍼레이터 (김도윤)
     *   RT       → 슈터 정방향 발사
     *   LT       → 슈터 역방향 구동
     *   D-pad ↑  → 슈터 목표 속도 +5%
     *   D-pad ↓  → 슈터 목표 속도 -5%
     *   B        → 컨베이어 지속 구동
     */
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutoChooser();
    }

    private void configureAutoChooser() {
        // 오토 0번: 드라이브 전진 + 인테이크 (기본값)
        m_autoChooser.setDefaultOption("Auto 0 - Drive + Intake (송주원)",
            new Auto0(m_drive, m_intakeRoller));
        // 오토 1번: 슈터 + 컨베이어
        m_autoChooser.addOption("Auto 1 - Shooter + Conveyor (김도윤)",
            new Auto1(m_shooter, m_conveyor));

        // PathPlanner 오토 추가 시도 (실패해도 무시)
        try {
            AutoBuilder.buildAutoChooser().getOptions().forEach((name, cmd) -> {
                if (!name.equals("None")) m_autoChooser.addOption(name, cmd);
            });
        } catch (RuntimeException e) {
            DriverStation.reportWarning("PathPlanner AutoBuilder not configured", false);
        }

        SmartDashboard.putData("Auto Mode", m_autoChooser);
    }

    private void configureBindings() {

        // ── 드라이브 기본 커맨드 (탱크 드라이브) ─────────────────
        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getRightX()
            )
        );

        // ── 슈터: 오퍼레이터 RT=정방향, LT=역방향, D-pad ±5% ─────
        m_shooter.setDefaultCommand(
            new ShooterCommand(
                m_shooter,
                m_operatorController::getRightTriggerAxis,
                m_operatorController::getLeftTriggerAxis,
                () -> m_operatorController.getHID().getPOV() == 0,
                () -> m_operatorController.getHID().getPOV() == 180
            )
        );

        // ── RT: AutoAlign (AprilTag ID 10 or 26, 방향만 정렬) ────
        m_driverController.rightTrigger(0.1)
            .whileTrue(new AutoAlignCommand(m_drive, m_vision));

        // ── 인테이크: LB=흡입(정방향), RB=토출(역방향) ───────────
        m_driverController.leftBumper()
            .whileTrue(new IntakeRollerCommand(m_intakeRoller, false));
        m_driverController.rightBumper()
            .whileTrue(new IntakeRollerCommand(m_intakeRoller, true));

        // ── 피벗: Y=UP / A=DOWN ──────────────────────────────────
        m_driverController.y().whileTrue(
            new IntakePivotCommand(m_intakePivot, Direction.UP)
        );
        m_driverController.a().whileTrue(
            new IntakePivotCommand(m_intakePivot, Direction.DOWN)
        );

        // ── 컨베이어: 오퍼레이터 B 버튼 지속 구동 ────────────────
        m_operatorController.b().whileTrue(new ConveyorCommand(m_conveyor));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void resetGyro() {
        m_drive.zeroGyro();
    }
}
