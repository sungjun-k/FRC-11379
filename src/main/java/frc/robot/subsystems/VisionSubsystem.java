package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    /**
     * PhotonVision 대시보드의 카메라 이름과 반드시 일치시켜야 한다.
     * 대시보드 접속 후 Cameras 탭에서 카메라 이름 확인 후 수정하세요.
     */
    private final PhotonCamera m_camera = new PhotonCamera("OV9281");

    /**
     * 로봇 중심(바닥 중앙) 기준 카메라까지의 오프셋
     * ★ 실제 측정값으로 반드시 수정할 것! (단위: 미터, 라디안)
     *   Translation3d(X앞뒤, Y좌우, Z위아래)
     *   Rotation3d(롤, 피치, 요우) - 정면을 바라보면 모두 0
     */
    private static final Transform3d CAMERA_OFFSET = new Transform3d(
        new Translation3d(0.0, 0.0, 0.5),   // 가정: 로봇 중심에서 위로 0.5m
        new Rotation3d(0, 0, 0)
    );

    private final PhotonPoseEstimator m_photonEstimator;

    public VisionSubsystem() {
        // kDefaultField: WPILib가 현재 시즌 기준 최신 필드를 자동으로 가리킨다.
        // k2026Reefscape 는 2025 시즌 상수라 2026 환경에서 존재하지 않으므로
        // 시즌 고유 상수를 쓰고 싶다면 IDE 자동완성으로 k2026... 항목을 확인하세요.
        AprilTagFieldLayout layout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        m_photonEstimator = new PhotonPoseEstimator(
            layout,
            // 여러 태그를 동시에 보이면 정확도가 높아진다
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CAMERA_OFFSET
        );
    }

    /**
     * 칼만 필터에 넣을 비전 측정 결과 반환.
     * 타임스탬프가 포함되어 있어 시간 지연 보정이 자동 적용된다.
     * AprilTag가 보이지 않으면 Optional.empty() 반환.
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return m_photonEstimator.update(m_camera.getLatestResult());
    }

    /** AprilTag 타겟이 시야에 있는지 여부 */
    public boolean hasTarget() {
        return m_camera.getLatestResult().hasTargets();
    }

    /**
     * 카메라 중심 기준 좌우 각도 오차 (Yaw, degree)
     * 양수 = 타겟이 왼쪽, 음수 = 타겟이 오른쪽
     * 타겟이 없으면 0.0 반환
     */
    public double getTargetYaw() {
        var result = m_camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }
}
