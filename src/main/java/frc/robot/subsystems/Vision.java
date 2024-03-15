package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Cameras;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
	PhotonCamera cameraMu, cameraNu, cameraXi;
	PhotonPoseEstimator estimatorMu, estimatorNu, estimatorXi;
	AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
	List<Pose3d> estimatedPoses;

	public Vision() {
		cameraMu = new PhotonCamera("Mu");
		cameraNu = new PhotonCamera("Nu");
		cameraXi = new PhotonCamera("Xi");

		estimatorMu = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraMu,
				Cameras.robotToCameraMu);
		estimatorNu = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraNu,
				Cameras.robotToCameraNu);
		estimatorXi = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraXi,
				Cameras.robotToCameraXi);
	}

	@Override
	public void periodic() {
		estimatedPoses = List.of(getEstimatedPose(estimatorMu.update()).estimatedPose,
				getEstimatedPose(estimatorNu.update()).estimatedPose,
				getEstimatedPose(estimatorXi.update()).estimatedPose);

		Logger.recordOutput("EstimatedPoses/Mu", estimatedPoses.get(0));
		Logger.recordOutput("EstimatedPoses/Nu", estimatedPoses.get(1));
		Logger.recordOutput("EstimatedPoses/Xi", estimatedPoses.get(2));

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

	public EstimatedRobotPose getEstimatedPose(Optional<EstimatedRobotPose> optionalEstimatePose) {
		if (optionalEstimatePose.isPresent()) {
			return optionalEstimatePose.get();
		} else {
			return null;
		}
	}
}
