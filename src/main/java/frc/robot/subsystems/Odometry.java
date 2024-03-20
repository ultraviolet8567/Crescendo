package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Cameras;
import frc.robot.Constants.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Odometry extends SubsystemBase {
	private static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
	/**
	 * Standard deviations of model states. Increase these numbers to trust your
	 * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ, with
	 * units in meters and radians, then meters.
	 */
	private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
	/**
	 * Standard deviations of the vision measurements. Increase these numbers to
	 * trust global measurements from vision less. This matrix is in the form [x, y,
	 * theta]ᵀ, with units in meters and radians.
	 */
	private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

	private Swerve swerve;

	private SwerveDrivePoseEstimator poseEstimator;

	private Pigeon2 gyro;
	private SwerveDriveOdometry odometer;

	// TODO: Put estimatorMu back when back-straight camera wired
	private PhotonCamera cameraNu, cameraXi;
	private PhotonPoseEstimator estimatorNu, estimatorXi;

	public Odometry(Swerve swerve) {
		System.out.println("[Init] Creating Odometry");

		this.swerve = swerve;

		/* Gyro */
		gyro = new Pigeon2(31);
		gyro.reset();

		odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyrometerHeading(),
				swerve.getModulePositions(), Constants.speaker);

		/* Vision */
		// "Mu", ID is 70, location
		// cameraMu = new PhotonCamera("BackStraight");
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
				Cameras.robotToCameraNu);
		estimatorXi = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraXi,
				Cameras.robotToCameraXi);

		// estimatorMu.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		estimatorNu.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		estimatorXi.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
		List<EstimatedRobotPose> estimatedPoses = getVisionEstimatedPoses();

		int counter = 0;
		for (EstimatedRobotPose estimatedPose : estimatedPoses) {
			poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);

			Logger.recordOutput("VisionPoses/" + counter, estimatedPoses.get(counter).estimatedPose.toPose2d());
			counter++;
		}

		/* Odometry */
		poseEstimator.update(gyro.getRotation2d(), swerve.getModulePositions());
	}

	public List<EstimatedRobotPose> getVisionEstimatedPoses() {
		List<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

		// TODO: Put estimatorMu back when back-straight camera wired
		for (PhotonPoseEstimator estimator : List.of(estimatorNu, estimatorXi)) {
			Optional<EstimatedRobotPose> optionalEstimatedPose = estimator.update();
			if (optionalEstimatedPose.isPresent()) {
				estimatedPoses.add(optionalEstimatedPose.get());
			}
		}

		return estimatedPoses;
	}

	public Rotation2d getSpeakerHeading() {
		int centerSpeakerTagID = (Constants.alliance == Alliance.Blue) ? 7 : 4;

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
