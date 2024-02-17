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
	private Pigeon2 gyro;
	private Swerve swerve;
	private SwerveDriveOdometry odometer;
	// pose2d will need to be updated with the estimated pose of the robot
	private static final Pose2d initialPose = new Pose2d();

	public Gyrometer(Swerve swerve) {
		this.swerve = swerve;
		gyro = new Pigeon2(31);
		gyro.reset();
		odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getHeading(), swerve.getModulePositions(),
				initialPose);
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Odometry/Pose", getPose());
		Logger.recordOutput("Odometry/Heading", Math.IEEEremainder(getHeading().getRadians(), 2 * Math.PI));

		update();
	}

	// returns pose
	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	public void update() {
		odometer.update(getHeading(), swerve.getModulePositions());
	}

	// getRotation2d() is CCW+ for the pigeon2, getAngle is CCW-
	public Rotation2d getHeading() {
		return gyro.getRotation2d();
	}

	// return gyro heading as rotation3d
	public Rotation3d getRotation3d() {
		return gyro.getRotation3d();
		// return gyro.getRotation3d().unaryMinus(); <-- if we need the negative version
	}

	// rate in degrees per second, negate to make CCW positive
	public double getRate() {
		return -gyro.getRate();
	}

	// reset odometry
	public void reset() {
		gyro.reset();
		odometer.resetPosition(getHeading(), swerve.getModulePositions(), initialPose);
	}
}
