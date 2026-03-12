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
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final VisionSubsystem       m_vision        = new VisionSubsystem();
    private final DriveSubsystem        m_drive         = new DriveSubsystem(m_vision);
    private final ShooterSubsystem      m_shooter       = new ShooterSubsystem();
    private final IntakeRollerSubsystem m_intakeRoller  = new IntakeRollerSubsystem();
    private final IntakePivotSubsystem  m_intakePivot   = new IntakePivotSubsystem();
    private final ConveyorSubsystem     m_conveyor      = new ConveyorSubsystem();

    /**
     * Port 0: Driver (송주원)
     *   Left Y / Right X -> Tank Drive
     *   RT  -> AutoAlign (AprilTag ID 10 or 26)
     *   LB  -> Intake forward (intake)
     *   RB  -> Intake reverse (eject)
     *   Y   -> Pivot UP   A -> Pivot DOWN
     */
    private final CommandXboxController m_driverController =
        new CommandXboxController(DRIVER_CONTROLLER_PORT);

    /**
     * Port 1: Operator (김도윤)
     *   RT       -> Shooter forward
     *   LT       -> Shooter reverse
     *   D-pad Up -> Shooter +5%   Down -> -5%
     *   B        -> Conveyor run
     */
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OPERATOR_CONTROLLER_PORT);

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

        m_drive.setDefaultCommand(
            new DriveCommand(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getRightX()
            )
        );

        m_shooter.setDefaultCommand(
            new ShooterCommand(
                m_shooter,
                m_operatorController::getRightTriggerAxis,
                m_operatorController::getLeftTriggerAxis,
                () -> m_operatorController.getHID().getPOV() == 0,
                () -> m_operatorController.getHID().getPOV() == 180
            )
        );

        // RT: AutoAlign (AprilTag ID 10 or 26, yaw only)
        m_driverController.rightTrigger(0.1)
            .whileTrue(new AutoAlignCommand(m_drive, m_vision));

        // LB: Intake forward   RB: Intake reverse
        m_driverController.leftBumper()
            .whileTrue(new IntakeRollerCommand(m_intakeRoller, false));
        m_driverController.rightBumper()
            .whileTrue(new IntakeRollerCommand(m_intakeRoller, true));

        // Y: Pivot UP   A: Pivot DOWN
        m_driverController.y().whileTrue(
            new IntakePivotCommand(m_intakePivot, Direction.UP)
        );
        m_driverController.a().whileTrue(
            new IntakePivotCommand(m_intakePivot, Direction.DOWN)
        );

        // Operator B: Conveyor continuous
        m_operatorController.b().whileTrue(new ConveyorCommand(m_conveyor));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void resetGyro() {
        m_drive.zeroGyro();
    }
}
