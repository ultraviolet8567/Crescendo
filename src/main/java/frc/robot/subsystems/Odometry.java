package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class Odometry extends SubsystemBase {
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
	private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

	private Gyrometer gyro;
	private Swerve swerve;
	private Vision vision;

	private SwerveDrivePoseEstimator poseEstimator;

	public Odometry(Gyrometer gyro, Swerve swerve, Vision vision) {
		this.gyro = gyro;
		this.swerve = swerve;
		this.vision = vision;

		poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getHeading(),
				swerve.getModulePositions(), new Pose2d(), STATE_STDS, VISION_STDS);
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		Logger.recordOutput("Odometry/Pose", getPose());

		List<EstimatedRobotPose> estimatedPoses = vision.getEstimatedPoses();

		int counter = 0;
		for (EstimatedRobotPose estimatedPose : estimatedPoses) {
			poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);

			Logger.recordOutput("EstimatedPoses/" + counter, estimatedPoses.get(counter).estimatedPose);
			counter++;
		}

		poseEstimator.update(gyro.getHeading(), swerve.getModulePositions());
	}

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}
}
