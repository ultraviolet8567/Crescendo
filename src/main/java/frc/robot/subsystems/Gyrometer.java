package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Gyrometer extends SubsystemBase {
	private static final Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));

	private Swerve swerve;

	private Pigeon2 gyro;
	private SwerveDriveOdometry odometer;

	public Gyrometer(Swerve swerve) {
		this.swerve = swerve;

		gyro = new Pigeon2(31);
		gyro.reset();

		odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getHeading(), swerve.getModulePositions(),
				initialPose);
		// odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
		// getHeading(), swerve.getModulePositions(),
		// new Pose2d(vision.getDistance().getTranslation().toTranslation2d(),
		// vision.getDistance().getRotation().toRotation2d()));
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Odometry/Pose", getPose());
		Logger.recordOutput("Odometry/Heading", Math.IEEEremainder(getHeading().getRadians(), 2 * Math.PI));

		update();
	}

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	public void update() {
		odometer.update(getHeading(), swerve.getModulePositions());
	}

	// return inverted pose
	public Pose2d getInvertedPose() {
		return new Pose2d(-getPose().getX(), -getPose().getY(), getPose().getRotation());
	}

	public Rotation2d getHeading() {
		// getRotation2d() is CCW+ for the Pigeon, getAngle is CCW-
		return gyro.getRotation2d();
	}

	public Rotation3d getRotation3d() {
		return gyro.getRotation3d();
		// return gyro.getRotation3d().unaryMinus(); <-- if we need the negative version
	}

	public double getRate() {
		// Rate of rotation (degrees/sec), negate for CCW+
		return -gyro.getRate();
	}

	public void resetPose(Pose2d pose) {
		odometer.resetPosition(getHeading(), swerve.getModulePositions(), pose);
		// odometer.resetPosition(getHeading(), swerve.getModulePositions(),
		// new Pose2d(vision.getDistance().getTranslation().toTranslation2d(),
		// vision.getDistance().getRotation().toRotation2d()));
	}

	public void reset() {
		gyro.reset();
		resetPose(initialPose);
	}
}
