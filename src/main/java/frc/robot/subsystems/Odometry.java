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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
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

	private PhotonCamera cameraStraight, cameraRight, cameraLeft;
	private PhotonPoseEstimator estimatorStraight, estimatorRight, estimatorLeft;

	public Odometry(Swerve swerve) {
		System.out.println("[Init] Creating Odometry");

		this.swerve = swerve;

		/* Gyro */
		gyro = new Pigeon2(31);
		gyro.reset();

		odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyrometerHeading(),
				swerve.getModulePositions(), AllianceFlipUtil.apply(new Pose2d(1.5, 5.5, new Rotation2d())));

		/* Vision */
		// "Mu", ID is 70, back middle
		// cameraStraight = new PhotonCamera("BackStraight");
		// cameraStraight.setDriverMode(false);
		// "Nu", ID is 71, back right
		cameraRight = new PhotonCamera("BackRight");
		cameraRight.setDriverMode(false);
		// "Xi", ID is 72, back left
		cameraLeft = new PhotonCamera("BackLeft");
		cameraLeft.setDriverMode(false);

		// estimatorStraight = new PhotonPoseEstimator(fieldLayout,
		// PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraStraight,
		// Cameras.robotToCameraMu);
		estimatorRight = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraRight,
				CameraConstants.kRobotToCameraRight);
		estimatorLeft = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraLeft,
				CameraConstants.kRobotToCameraLeft);

		// estimatorStraight.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		estimatorRight.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		estimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

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

		/* Odometry */
		poseEstimator.update(gyro.getRotation2d(), swerve.getModulePositions());

		/* Vision */
		Pose2d visionPose = updateVisionPoses();
		Pose2d odometerPose = odometer.getPoseMeters();

		if (visionPose.getTranslation()
				.getDistance(odometerPose.getTranslation()) > OdometryConstants.kOdometerDriftCorrection) {
			resetOdometerPose(visionPose);
		}

	}

	// public List<EstimatedRobotPose> getVisionEstimatedPoses() {
	// List<EstimatedRobotPose> estimatedPoses = new
	// ArrayList<EstimatedRobotPose>();

	// // TODO: Put estimatorStraight back when back-straight camera wired
	// for (PhotonPoseEstimator estimator : List.of(estimatorRight, estimatorLeft))
	// {
	// Optional<EstimatedRobotPose> optionalEstimatedPose = estimator.update();
	// if (optionalEstimatedPose.isPresent()) {
	// estimatedPoses.add(optionalEstimatedPose.get());
	// }
	// }

	// return estimatedPoses;
	// }

	public Pose2d updateVisionPoses() {
		Optional<EstimatedRobotPose> estimatedPosesRight = estimatorRight.update();
		estimatorRight.setReferencePose(poseEstimator.getEstimatedPosition());
		Optional<EstimatedRobotPose> estimatedPosesLeft = estimatorLeft.update();
		estimatorLeft.setReferencePose(poseEstimator.getEstimatedPosition());

		List<Pose3d> optionsRight = new ArrayList<Pose3d>();
		List<Pose3d> optionsLeft = new ArrayList<Pose3d>();

		// Create a list of Pose3d options for back right camera
		if (estimatedPosesRight.isPresent()) {
			if (estimatedPosesRight.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
				optionsRight.add(estimatedPosesRight.get().estimatedPose);
			} else {
				optionsRight = getAmbiguousPoses(cameraRight.getLatestResult(), CameraConstants.kRobotToCameraRight);
			}
		}

		// Create a list of Pose3d options for the back left camera
		if (estimatedPosesLeft.isPresent()) {
			if (estimatedPosesLeft.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
				optionsLeft.add(estimatedPosesLeft.get().estimatedPose);
			} else {
				optionsLeft = getAmbiguousPoses(cameraLeft.getLatestResult(), CameraConstants.kRobotToCameraLeft);
			}
		}

		Pose2d bestRightPose = new Pose2d();
		Pose2d bestLeftPose = new Pose2d();
		double minDistance = Double.MAX_VALUE;

		if (!estimatedPosesRight.isPresent() && estimatedPosesLeft.isPresent()) {
			bestRightPose = estimatedPosesLeft.get().estimatedPose.toPose2d();
			poseEstimator.addVisionMeasurement(bestRightPose, estimatedPosesRight.get().timestampSeconds);
		} else if (!estimatedPosesLeft.isPresent() && estimatedPosesRight.isPresent()) {
			bestLeftPose = estimatedPosesRight.get().estimatedPose.toPose2d();
			poseEstimator.addVisionMeasurement(bestLeftPose, estimatedPosesLeft.get().timestampSeconds);
		} else if (estimatedPosesRight.isPresent() && estimatedPosesLeft.isPresent()) {
			// compare all left and right to each other to find correct robot pose
			for (Pose3d poseRight : optionsRight) {
				for (Pose3d poseLeft : optionsLeft) {
					double distance = calculateDifference(poseRight, poseLeft);

					// Makes the smallest difference the measurement
					if (distance < minDistance) {
						bestRightPose = poseRight.toPose2d();
						bestLeftPose = poseLeft.toPose2d();
						minDistance = distance;
					}
				}
			}

			poseEstimator.addVisionMeasurement(bestRightPose, estimatedPosesRight.get().timestampSeconds);
			poseEstimator.addVisionMeasurement(bestLeftPose, estimatedPosesLeft.get().timestampSeconds);
		}

		Logger.recordOutput("Odometry/Vision/LeftPose", bestLeftPose);
		Logger.recordOutput("Odometry/Vision/RightPose", bestRightPose);

		double xSum = bestLeftPose.getX() + bestRightPose.getX();
		double ySum = bestLeftPose.getY() + bestRightPose.getY();

		return new Pose2d(xSum / 2, ySum / 2, bestLeftPose.getRotation());
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
		return FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d().minus(getPose().getTranslation())
				.getAngle();
	}

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Pose2d getOdometerPose() {
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

	public void resetOdometerPose(Pose2d pose) {
		odometer.resetPosition(getGyrometerHeading(), swerve.getModulePositions(), pose);
	}

	public void resetGyrometerHeading() {
		gyro.reset();
	}
}
