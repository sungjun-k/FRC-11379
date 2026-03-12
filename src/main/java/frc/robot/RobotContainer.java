package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.Auto0;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.IntakePivotCommand.Direction;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TagDistanceHoldCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final VisionSubsystem       m_vision       = new VisionSubsystem();
    private final DriveSubsystem        m_drive        = new DriveSubsystem(m_vision);
    private final ShooterSubsystem      m_shooter      = new ShooterSubsystem();
    private final IntakeRollerSubsystem m_intakeRoller = new IntakeRollerSubsystem();
    private final IntakePivotSubsystem  m_intakePivot  = new IntakePivotSubsystem();
    private final ConveyorSubsystem     m_conveyor     = new ConveyorSubsystem();

    // Port 0: 드라이버 (송주원)   Port 1: 오퍼레이터 (김도윤)
    private final CommandXboxController m_driver   = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController m_operator = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutoChooser();
    }

    private void configureAutoChooser() {
        m_autoChooser.setDefaultOption(
            "Auto 0 - Pivot Down > Reverse > Shoot",
            new Auto0(m_drive, m_intakePivot, m_shooter, m_conveyor)
        );
        try {
            Command ppAuto = AutoBuilder.buildAutoChooser().getSelected();
            if (ppAuto != null) m_autoChooser.addOption("PathPlanner Auto", ppAuto);
        } catch (RuntimeException e) {
            DriverStation.reportWarning("PathPlanner AutoBuilder not configured", false);
        }
        SmartDashboard.putData("Auto Mode", m_autoChooser);
    }

    private void configureBindings() {

        // ━━ Driver (Port 0, 송주원) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // Left Y / Right X → 탱크 드라이브
        m_drive.setDefaultCommand(new DriveCommand(
            m_drive,
            () -> -m_driver.getLeftY(),
            () -> -m_driver.getRightX()
        ));

        // RT (0.1이상) → AutoAlign AprilTag ID 10 or 26
        m_driver.rightTrigger(0.1).whileTrue(new AutoAlignCommand(m_drive, m_vision));

        // LB → 인테이크 흡입   RB → 인테이크 토출
        m_driver.leftBumper() .whileTrue(new IntakeRollerCommand(m_intakeRoller, false));
        m_driver.rightBumper().whileTrue(new IntakeRollerCommand(m_intakeRoller, true));

        // Y → 피벗 UP   A → 피벗 DOWN
        m_driver.y().whileTrue(new IntakePivotCommand(m_intakePivot, Direction.UP));
        m_driver.a().whileTrue(new IntakePivotCommand(m_intakePivot, Direction.DOWN));

        // ━━ Operator (Port 1, 김도윤) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        // RB → 슈터 정방향   RT → 슈터 역방향   D-pad ±5%
        m_shooter.setDefaultCommand(new ShooterCommand(
            m_shooter,
            () -> m_operator.getHID().getRightBumper(),
            m_operator::getRightTriggerAxis,
            () -> m_operator.getHID().getPOV() == 0,
            () -> m_operator.getHID().getPOV() == 180
        ));

        // B → 컨베이어 패턴 (0.15초 ON / 0.40초 OFF)
        m_operator.b().whileTrue(new ConveyorCommand(m_conveyor));
    }

    public Command getAutonomousCommand()         { return m_autoChooser.getSelected(); }
    public TagDistanceHoldCommand createDistHoldCommand() {
        return new TagDistanceHoldCommand(m_drive, m_vision);
    }
    public void resetGyro()    { m_drive.zeroGyro(); }
    public void pivotDisable() { m_intakePivot.disablePID(); }
}
