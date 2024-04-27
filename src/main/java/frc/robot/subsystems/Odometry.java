package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Odometry extends SubsystemBase {
	private static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
	/**
	 * Standard deviations of model states. Increase these numbers to trust your
	 * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ, with
	 * units in meters and radians, then meters.
	 */
	private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
	/**
	 * Standard deviations of the vision measurements. Increase these numbers to
	 * trust global measurements from vision less. This matrix is in the form [x, y,
	 * theta]ᵀ, with units in meters and radians.
	 */
	private static final Vector<N3> VISION_STDS = VecBuilder.fill(5, 5, Units.degreesToRadians(500));

	private Swerve swerve;
	private SwerveDrivePoseEstimator poseEstimator;

	private Pigeon2 gyro;
	private SwerveDriveOdometry odometer;

	private PhotonCamera cameraMu, cameraNu, cameraXi;
	private PhotonPoseEstimator estimatorMu, estimatorNu, estimatorXi;

	public Odometry(Swerve swerve) {
		System.out.println("[Init] Creating Odometry");

		this.swerve = swerve;

		/* Gyro */
		gyro = new Pigeon2(31);
		gyro.reset();

		odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyrometerHeading(),
				swerve.getModulePositions(), Constants.speaker);

		/* Vision */
		// "Mu", ID is 70, back middle
		// cameraMu = new PhotonCamera("BackStraight");
		// cameraMu.setDriverMode(false);
		// "Nu", ID is 71, back right
		cameraNu = new PhotonCamera("BackRight");
		cameraNu.setDriverMode(false);
		// "Xi", ID is 72, back left
		cameraXi = new PhotonCamera("BackLeft");
		cameraXi.setDriverMode(false);

		// estimatorMu = new PhotonPoseEstimator(fieldLayout,
		// PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraMu,
		// Cameras.robotToCameraMu);
		estimatorNu = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraNu,
				CameraConstants.kRobotToCameraNu);
		estimatorXi = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraXi,
				CameraConstants.kRobotToCameraXi);

		// estimatorMu.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		estimatorNu.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		estimatorXi.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

		/* Odometry */
		poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getRotation2d(),
				swerve.getModulePositions(), new Pose2d(), STATE_STDS, VISION_STDS);
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		Logger.recordOutput("Odometry/Pose", getPose());
		Logger.recordOutput("Odometry/Heading", getHeading());

		Logger.recordOutput("Gyrometer/Pose", odometer.getPoseMeters());
		Logger.recordOutput("Gyrometer/Heading", gyro.getRotation2d());

		/* Gyro */
		odometer.update(getGyrometerHeading(), swerve.getModulePositions());

		/* Vision */
		// List<EstimatedRobotPose> estimatedPoses = getVisionEstimatedPoses();

		// int counter = 0;
		// for (EstimatedRobotPose estimatedPose : estimatedPoses) {
		// poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
		// estimatedPose.timestampSeconds);

		// Logger.recordOutput("VisionPoses/" + counter,
		// estimatedPoses.get(counter).estimatedPose.toPose2d());
		// counter++;
		// }

		updateVisionPoses();

		/* Odometry */
		poseEstimator.update(gyro.getRotation2d(), swerve.getModulePositions());
	}

	// public List<EstimatedRobotPose> getVisionEstimatedPoses() {
	// List<EstimatedRobotPose> estimatedPoses = new
	// ArrayList<EstimatedRobotPose>();

	// // TODO: Put estimatorMu back when back-straight camera wired
	// for (PhotonPoseEstimator estimator : List.of(estimatorNu, estimatorXi)) {
	// Optional<EstimatedRobotPose> optionalEstimatedPose = estimator.update();
	// if (optionalEstimatedPose.isPresent()) {
	// estimatedPoses.add(optionalEstimatedPose.get());
	// }
	// }

	// return estimatedPoses;
	// }

	public void updateVisionPoses() {
		Optional<EstimatedRobotPose> nuEstimatedPoses = estimatorNu.update();
		estimatorNu.setReferencePose(poseEstimator.getEstimatedPosition());
		Optional<EstimatedRobotPose> xiEstimatedPoses = estimatorXi.update();
		estimatorXi.setReferencePose(poseEstimator.getEstimatedPosition());

		List<Pose3d> nuOptions = new ArrayList<Pose3d>();
		List<Pose3d> xiOptions = new ArrayList<Pose3d>();

		// Create a list of Pose3d options for Nu/back right camera
		if (nuEstimatedPoses.isPresent()) {
			if (nuEstimatedPoses.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
				nuOptions.add(nuEstimatedPoses.get().estimatedPose);
			} else {
				nuOptions = getAmbiguousPoses(cameraNu.getLatestResult(), CameraConstants.kRobotToCameraNu);
			}
		}

		// Create a list of Pose3d options for the Xi/back left camera
		if (xiEstimatedPoses.isPresent()) {
			if (xiEstimatedPoses.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
				xiOptions.add(xiEstimatedPoses.get().estimatedPose);
			} else {
				xiOptions = getAmbiguousPoses(cameraXi.getLatestResult(), CameraConstants.kRobotToCameraXi);
			}
		}

		Pose3d bestNuPose3d = new Pose3d();
		Pose3d bestXiPose3d = new Pose3d();
		double minDistance = 1e6;

		if (!nuEstimatedPoses.isPresent() && xiEstimatedPoses.isPresent()) {
			Pose2d pose = xiEstimatedPoses.get().estimatedPose.toPose2d();
			poseEstimator.addVisionMeasurement(pose, cameraNu.getLatestResult().getTimestampSeconds());
		} else if (!xiEstimatedPoses.isPresent() && nuEstimatedPoses.isPresent()) {
			Pose2d pose = nuEstimatedPoses.get().estimatedPose.toPose2d();
			poseEstimator.addVisionMeasurement(pose, cameraNu.getLatestResult().getTimestampSeconds());
		} else if (nuEstimatedPoses.isPresent() && xiEstimatedPoses.isPresent()) {
			// compare all right poses and left poses to each other to find correct robot
			// pose
			for (Pose3d nuPose : nuOptions) {
				for (Pose3d xiPose : xiOptions) {
					double distance = calculateDifference(nuPose, xiPose);

					// makes the smallest difference the measurement
					if (distance < minDistance) {
						bestNuPose3d = nuPose;
						bestXiPose3d = xiPose;
						minDistance = distance;
					}
				}
			}

			poseEstimator.addVisionMeasurement(bestXiPose3d.toPose2d(),
					cameraNu.getLatestResult().getTimestampSeconds());
			poseEstimator.addVisionMeasurement(bestNuPose3d.toPose2d(),
					cameraXi.getLatestResult().getTimestampSeconds());
		}

		return;
	}

	private ArrayList<Pose3d> getAmbiguousPoses(PhotonPipelineResult result, Transform3d robotToCamera) {
		ArrayList<Pose3d> ambigiousPoses = new ArrayList<>();
		for (PhotonTrackedTarget target : result.targets) {
			int targetFiducialId = target.getFiducialId();

			// Don't report errors for non-fiducial targets. This could also be resolved by
			// adding -1 to
			// the initial HashSet.
			if (targetFiducialId == -1)
				continue;

			Optional<Pose3d> targetPosition = fieldLayout.getTagPose(target.getFiducialId());

			if (targetPosition.isEmpty())
				continue;

			// add all possible robot positions to the array that is returned
			ambigiousPoses.add(targetPosition.get().transformBy(target.getBestCameraToTarget().inverse())
					.transformBy(robotToCamera.inverse()));
			ambigiousPoses.add(targetPosition.get().transformBy(target.getAlternateCameraToTarget().inverse())
					.transformBy(robotToCamera.inverse()));
		}

		return ambigiousPoses;
	}

	private double calculateDifference(Pose3d x, Pose3d y) {
		return x.getTranslation().getDistance(y.getTranslation());
	}

	public Rotation2d getSpeakerHeading() {
		int centerSpeakerTagID = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? 7 : 4;

		return getPose().minus(fieldLayout.getTagPose(centerSpeakerTagID).get().toPose2d()).getRotation();
	}

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Pose2d getGyrometerPose() {
		return odometer.getPoseMeters();
	}

	public Rotation2d getHeading() {
		return poseEstimator.getEstimatedPosition().getRotation();
	}

	public Rotation2d getGyrometerHeading() {
		return gyro.getRotation2d();
	}

	public void setGyroYaw(Rotation2d yaw) {
		gyro.setYaw(yaw.getDegrees());
	}

	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(gyro.getRotation2d(), swerve.getModulePositions(), pose);
	}

	public void resetGyrometerPose(Pose2d pose) {
		odometer.resetPosition(getGyrometerHeading(), swerve.getModulePositions(), pose);
	}

	public void resetGyrometerHeading() {
		gyro.reset();
	}
}
