package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    // ── 모터 ──────────────────────────────────────────
    private final TalonFX m_leftFront  = new TalonFX(LEFT_FRONT_ID);
    private final TalonFX m_leftRear   = new TalonFX(LEFT_REAR_ID);
    private final TalonFX m_rightFront = new TalonFX(RIGHT_FRONT_ID);
    private final TalonFX m_rightRear  = new TalonFX(RIGHT_REAR_ID);

    private final DifferentialDrive m_drive = new DifferentialDrive(
        m_leftFront::set,
        m_rightFront::set
    );

    // ── 센서 ──────────────────────────────────────────
    // NavX2: roboRIO MXP 포트(SPI) 연결 기준
    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // ── 오도메트리 ────────────────────────────────────
    private final DifferentialDriveKinematics m_kinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    private final DifferentialDriveOdometry m_odometry;

    // ─────────────────────────────────────────────────
    public DriveSubsystem() {
        configureTalon(m_leftFront,  false);
        configureTalon(m_leftRear,   false);
        configureTalon(m_rightFront, true);  
        configureTalon(m_rightRear,  true);

        m_drive.setSafetyEnabled(false);

        // Rear → Front 팔로워 (v26 API)
        m_leftRear.setControl(new Follower(LEFT_FRONT_ID,   MotorAlignmentValue.Aligned));
        m_rightRear.setControl(new Follower(RIGHT_FRONT_ID, MotorAlignmentValue.Aligned));

        // 엔코더 0점 초기화
        m_leftFront.setPosition(0);
        m_rightFront.setPosition(0);

        // ✅ 수정: 초기화 시에도 부호가 뒤집힌 각도(getGyroRotation) 사용
        m_odometry = new DifferentialDriveOdometry(
            getGyroRotation(),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );

        // PathPlanner AutoBuilder 설정
        configurePathPlanner();
    }

    // ── PathPlanner 설정 ──────────────────────────────
    private void configurePathPlanner() {
        RobotConfig config;
        try {
            // PathPlanner GUI에서 설정한 로봇 물리 파라미터 로드
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            this::getPose,                     // 현재 위치 제공
            this::resetPose,                   // 위치 초기화 (오토 시작 시)
            this::getRobotRelativeSpeeds,      // 현재 로봇 상대 속도
            (speeds, feedforwards) -> driveRobotRelative(speeds), // 경로 추종 구동
            new PPLTVController(0.02),         // 차동구동용 LTV 컨트롤러 (dt=20ms)
            config,
            () -> {
                // 레드 얼라이언스이면 경로 반전
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent()
                    && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    // ── TalonFX 공통 설정 ─────────────────────────────
    private void configureTalon(TalonFX motor, boolean inverted) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = SUPPLY_CURRENT_LIMIT_AMPS;
        motor.getConfigurator().apply(config);
    }

    /** 
     * ✅ 핵심 수정: NavX의 각도를 WPILib 기준(반시계가 +)에 맞게 변환합니다.
     */
    private Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    // ── 주기적 오도메트리 업데이트 ───────────────────
    @Override
    public void periodic() {
        // ✅ 수정: 업데이트 시 부호가 뒤집힌 각도(getGyroRotation) 사용
        m_odometry.update(
            getGyroRotation(),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );

        var pose = m_odometry.getPoseMeters();
        SmartDashboard.putNumber("Odometry X (m)", pose.getX());
        SmartDashboard.putNumber("Odometry Y (m)", pose.getY());
        SmartDashboard.putNumber("Odometry Deg",  pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Left Dist (m)",  getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Dist (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Gyro Yaw (deg)", getGyroRotation().getDegrees()); // 변환된 각도 출력
    }

    // ── PathPlanner 필수 메서드 ───────────────────────

    /** 현재 필드 기준 위치 반환 */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    public void zeroGyro() {
        m_gyro.reset();
    }

    /** 오토 시작점 기준으로 오도메트리 초기화 */
    public void resetPose(Pose2d pose) {
        m_leftFront.setPosition(0);
        m_rightFront.setPosition(0);
        
        // ✅ 수정: resetPosition 시 부호가 뒤집힌 각도(getGyroRotation) 사용
        m_odometry.resetPosition(
            getGyroRotation(),
            0, 0,
            pose
        );
    }

    /** 로봇 상대 속도 반환 (PathPlanner 요구) */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                getLeftVelocityMS(),
                getRightVelocityMS()
            )
        );
    }

    /** PathPlanner에서 계산한 목표 속도로 구동 */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(MAX_SPEED_MPS);  
        m_drive.tankDrive(
            wheelSpeeds.leftMetersPerSecond  / MAX_SPEED_MPS,
            wheelSpeeds.rightMetersPerSecond / MAX_SPEED_MPS
        );
    }

    // ── 텔레옵 드라이브 ───────────────────────────────

    public void arcadeDrive(double xSpeed, double zRotation) {
        m_drive.arcadeDrive(
            xSpeed    * DRIVE_SPEED_SCALE,
            zRotation * DRIVE_ROTATION_SCALE
        );
    }

    public void stopDrive() {
        m_drive.stopMotor();
    }

    // ── 내부 엔코더 헬퍼 ──────────────────────────────

    private double getLeftDistanceMeters() {
        // Phoenix 6: getPosition() → 회전수(rot) → × 변환계수 → 미터(m)
        return m_leftFront.getPosition().getValueAsDouble() * ENCODER_POSITION_FACTOR;
    }

    private double getRightDistanceMeters() {
        return m_rightFront.getPosition().getValueAsDouble() * ENCODER_POSITION_FACTOR;
    }

    private double getLeftVelocityMS() {
        // Phoenix 6: getVelocity() → rps → × 변환계수 → m/s
        return m_leftFront.getVelocity().getValueAsDouble() * ENCODER_POSITION_FACTOR;
    }

    private double getRightVelocityMS() {
        return m_rightFront.getVelocity().getValueAsDouble() * ENCODER_POSITION_FACTOR;
    }
}
