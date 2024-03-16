package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Cameras;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
	private static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	private PhotonCamera cameraMu, cameraNu, cameraXi;
	private PhotonPoseEstimator estimatorMu, estimatorNu, estimatorXi;

	public Vision() {
		// Mu ID is 70, location
		cameraMu = new PhotonCamera("Mu");
		// Nu ID is 71, back right
		cameraNu = new PhotonCamera("Nu");
		// Xi ID is 72, back left
		cameraXi = new PhotonCamera("Xi");

		estimatorMu = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraMu,
				Cameras.robotToCameraMu);
		estimatorNu = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraNu,
				Cameras.robotToCameraNu);
		estimatorXi = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraXi,
				Cameras.robotToCameraXi);

		estimatorMu.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		estimatorNu.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		estimatorXi.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	@Override
	public void periodic() {
		// 🐖 <-- Carlos II reincarnate (DO NOT REMOVE)
		// var result = cameraMu.getLatestResult();
		// if (result.hasTargets()) {
		// List<PhotonTrackedTarget> target = result.getTargets();
		// PhotonTrackedTarget bestTarget = result.getBestTarget();

		// Transform3d poseTransform = bestTarget.getBestCameraToTarget();
		// Transform3d poseTransformAlternate = bestTarget.getAlternateCameraToTarget();

		// double yaw = bestTarget.getYaw();
		// int targetID = bestTarget.getFiducialId();
		// double poseAmbiguity = bestTarget.getPoseAmbiguity();

		// Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(poseTransform,
		// fieldLayout.getTagPose(bestTarget.getFiducialId()).get(),
		// Cameras.robotToCameraMu.inverse());

		// camera.takeInputSnapshot();
		// camera.takeOutputSnapshot();
		// }
	}

	public List<EstimatedRobotPose> getEstimatedPoses() {
		List<EstimatedRobotPose> estimatedPoses = Collections.emptyList();

		for (PhotonPoseEstimator estimator : List.of(estimatorMu, estimatorNu, estimatorXi)) {
			Optional<EstimatedRobotPose> optionalEstimatedPose = estimator.update();
			if (optionalEstimatedPose.isPresent()) {
				estimatedPoses.add(optionalEstimatedPose.get());
			}
		}

		return estimatedPoses;
	}
}
