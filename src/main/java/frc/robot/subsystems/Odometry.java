// package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
	private Vision vision;
	private Gyrometer gyro;
	private double gyroWeight = 0.6;
	private double visionWeight = 0.4;

	public Odometry(Vision vision, Gyrometer gyro) {
		this.vision = vision;
		this.gyro = gyro;
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		gyro.update();
		vision.update();
	}

	// calculate weighted average 
	public double getAverage() {
		return add(vision.getDistance(), visionWeight) + add(gyro.getPose(), gyroWeight);
	}

	public Rotation3d getHeading() {
		double roll = average(vision.getDistance().getRotation().getX(), gyro.getRotation3d().getX());
		double pitch = average(vision.getDistance().getRotation().getY(), gyro.getRotation3d().getY());
		double yaw = average(vision.getDistance().getRotation().getZ(), gyro.getRotation3d().getZ());

		return new Rotation3d(roll, pitch, yaw);
	}

	private double add(Transform3d t, double value) {
		return (t.getX() * value) + (t.getY() * value);
	}

	private double add(Pose2d p, double value) {
		return (p.getX() * value) + (p.getY() * value);
	}

	private double average(double one, double two) {
		return (one + two) / 2;
	}

	public Rotation2d getRotation2d() {
		return new Rotation2d(); 
	}
}
