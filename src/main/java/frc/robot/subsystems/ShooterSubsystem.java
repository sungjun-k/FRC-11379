package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;          
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX m_motor5 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_5_ID);
    private final TalonFX m_motor6 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_6_ID);
    private final TalonFX m_motor7 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_7_ID);

    public ShooterSubsystem() {
        // 5, 6번 공통 config (정방향)
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // 7번 전용 config (역방향 반전)
        TalonFXConfiguration config7 = new TalonFXConfiguration();
        config7.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config7.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive; 
        config7.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.SUPPLY_CURRENT_LIMIT_AMPS;
        config7.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_motor5.getConfigurator().apply(config); 
        m_motor6.getConfigurator().apply(config);
        m_motor7.getConfigurator().apply(config7); 
    }

    /**
     * 5번, 7번 모터만 출력 설정 (6번은 제외)
     */
    public void setOutput(double output) {
        m_motor5.set(output);
        // m_motor6.set(output);  // 6번 모터는 제외
        m_motor7.set(output);
    }
    
    /**
     * 6번 모터만 고정 속도로 구동 (0.8)
     */
    public void setMotor6Fixed(double fixedSpeed) {
        m_motor6.set(fixedSpeed);
    }

    public void stop() {
        m_motor5.set(0.0);
        m_motor6.set(0.0);
        m_motor7.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Motor5 Output", m_motor5.get());
        SmartDashboard.putNumber("Shooter/Motor6 Output", m_motor6.get());
        SmartDashboard.putNumber("Shooter/Motor7 Output", m_motor7.get());
    }
}
