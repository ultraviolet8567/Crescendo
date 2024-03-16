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
		// im comin for you carlos >:D 🔪

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

	// THIS CODE IS IN NEED OF COMPATIBILITY CHANGES
	//
	// CODE PROPERTIES:
	// 1) This uses an XYZ system where Z is the up axis
	// 2) Purely mathematical, no interface with data from the robot
	// 3) Completely untested, uses math imported from https://www.desmos.com/3d/606011a56d

	public void setPose() {
		// Interface code
	}

	public double solveBodyRot(double xDiff, double yDiff) {
		return Math.atan2(yDiff,xDiff);
	}

	public double solveShooterRot(double xDiff, double zDiff, double exitVel, boolean isUpper) {
		//a_{1}=\arctan\left(\frac{-T.x+\sqrt{T.x^{2}-4\left(\frac{gT.x^{2}}{v^{2}}\cdot\left(\frac{gT.x^{2}}{v^{2}}-T.y\right)\right)}}{2\cdot\frac{gT.x^{2}}{v^{2}}}\right)
		//\frac{gT.x^{2}}{v^{2}}
		
		double gravity = -9.8;
		double var = Math.pow(xDiff,2)*gravity/Math.pow(exitVel,2);
		double deriv = Math.pow(xDiff,2)-4*(var*(var-zDiff));

		if (deriv < 0) {
			return -999;
		}

		if (isUpper) {
			return Math.atan((-xDiff+Math.sqrt(deriv))/(2*var));
		}
		else {
			return Math.atan((-xDiff-Math.sqrt(deriv))/(2*var));
		}
	}
}
