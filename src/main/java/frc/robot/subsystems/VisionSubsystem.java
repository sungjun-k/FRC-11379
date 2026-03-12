package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera m_camera = new PhotonCamera("OV9281");

    private static final Transform3d CAMERA_OFFSET = new Transform3d(
        new Translation3d(0.0, 0.0, 0.5),
        new Rotation3d(0, 0, 0)
    );

    private final PhotonPoseEstimator m_photonEstimator;

    public VisionSubsystem() {
        var layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        m_photonEstimator = new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CAMERA_OFFSET
        );
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return m_photonEstimator.update(m_camera.getLatestResult());
    }

    /** 아무 AprilTag든 시야에 있으면 true */
    public boolean hasTarget() {
        return m_camera.getLatestResult().hasTargets();
    }

    /**
     * 가장 잘 보이는 타겟의 Yaw 반환.
     * 양수 = 타겟이 왼쪽, 음수 = 오른쪽. 없으면 0.0
     */
    public double getTargetYaw() {
        var result = m_camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    /**
     * 지정된 ID 중 하나가 보이면 해당 Yaw 반환.
     * 둘 다 보이면 가장 앞쪽(fiducialId 순서 무관, ambiguity 낮은) 것 우선.
     * 해당 ID가 없으면 Double.NaN 반환.
     *
     * @param ids 검색할 AprilTag ID 목록
     */
    public double getTargetYawById(int... ids) {
        var result = m_camera.getLatestResult();
        if (!result.hasTargets()) return Double.NaN;

        for (PhotonTrackedTarget t : result.getTargets()) {
            for (int id : ids) {
                if (t.getFiducialId() == id) {
                    return t.getYaw();
                }
            }
        }
        return Double.NaN;
    }
}
