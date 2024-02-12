package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
	Vision vision;
	Gyrometer gyro;

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

	/* Define all subsystem-specific methods and enums here */

	public Rotation2d getRotation2d() {
		return new Rotation2d(); // Placeholder value until we have odometry data
	}
}
