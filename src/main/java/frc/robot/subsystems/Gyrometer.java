package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Gyrometer extends SubsystemBase {
	private Pigeon2 gyro;
	private Swerve swerve;
	private Vision vision;
	private SwerveDriveOdometry odometer;

	public Gyrometer(Swerve swerve, Vision vision) {
		gyro = new Pigeon2(31);
		gyro.reset();

		this.swerve = swerve;
		this.vision = vision;
		odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getHeading(), swerve.getModulePositions(),
				new Pose2d(vision.getDistance().getTranslation().toTranslation2d(),
						vision.getDistance().getRotation().toRotation2d()));
	}

	@Override
	public void periodic() {
		odometer.update(getHeading(), swerve.getModulePositions());
	}

	// returns pose
	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	// return inverted pose
	public Pose2d getInvertedPose() {
		return new Pose2d(-getPose().getX(), -getPose().getY(), getPose().getRotation());
	}

	// return gyro heading as rotation2d
	// getRotation2d() is CCW+ for the pigeon2, getAngle is CCW-
	public Rotation2d getHeading() {
		return gyro.getRotation2d();
	}

	// return gyro heading as rotation3d
	public Rotation3d getRotation3d() {
		return gyro.getRotation3d();
		// return gyro.getRotation3d().unaryMinus(); <-- if we need the negative version
	}

	// rate in degrees per second, clockwise is positive
	public double getRate() {
		return gyro.getRate();
	}

	// reset odometry
	public void reset() {
		gyro.reset();
		odometer.resetPosition(getHeading(), swerve.getModulePositions(),
				new Pose2d(vision.getDistance().getTranslation().toTranslation2d(),
						vision.getDistance().getRotation().toRotation2d()));
	}
}
