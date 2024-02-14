/**
 * things that we could do now:
 * horizontal alignment (swerve)
 * id (which side of the field we're on) affects calculations --> will differ based on field-based/robot-based
 * - field: positive theta is counter-clockwise, positive x-axis is away from alliance wall, positive y-axis is perpendicular & to the left of positive x
 * - robot: positive theta is counter-clockwise, positive x-axis is dir robot is facing, positive y-axis is perpendicular & to the left of robot
*/

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
	private Camera right, left, back;
	private AprilTagFieldLayout field = Constants.fieldLayout;

	// TODO: fill in name & find the transform3ds
	public Vision(KMeans kmeans) {
		right = new Camera("", Constants.Cameras.frontRighttoRobot, kmeans, field);
		left = new Camera("", Constants.Cameras.frontLefttoRobot, kmeans, field);
		back = new Camera("", Constants.Cameras.backToRobot, kmeans, field);
	}

	@Override
	public void periodic() {
		update();
	}

	public Transform3d getDistance() {
		return new Transform3d(
			average(right.getDistance().getX(), left.getDistance().getX(), back.getDistance().getX()),
			average(right.getDistance().getY(), left.getDistance().getY(), back.getDistance().getY()),
			average(right.getDistance().getZ(), left.getDistance().getZ(), back.getDistance().getZ()),
			average(right.getDistance().getRotation(), left.getDistance().getRotation(), back.getDistance().getRotation())
		);
	}

	public Rotation2d getRotToAlign() {
		return average(
			right.getRotToAlign(right.getTargets()), 
			left.getRotToAlign(left.getTargets()), 
			back.getRotToAlign(back.getTargets())
		);
	}

	public double getShootVelocity() {
		return average(right.getShootVelocity(), left.getShootVelocity(), back.getShootVelocity());
	}

	// thank you so much stephanie
	public double[] getShootRad() {
		Transform3d d = getDistance();
		double g = -9.80665;
		double x2 = Math.pow(d.getX(), 2);
		double thing = (g * x2) / 2 * Math.pow(getShootVelocity(), 2);

		double minus = Math.atan(
			(d.getX()) + Math.sqrt(x2 - (4 * thing * (d.getY() + thing))) / (2 * thing)
		);

		double plus = Math.atan(
			(d.getX()) - Math.sqrt(x2 - (4 * thing * (d.getY() + thing))) / (2 * thing)
		);

		return new double[] {plus, minus};
	}

	public void update() {
		right.update();
		left.update();
		back.update();
	}

	private double average(double one, double two, double three) {
		return (one + two + three) / 3;
	}

	private Rotation3d average(Rotation3d one, Rotation3d two, Rotation3d three) {
		return new Rotation3d(
			average(one.getX(), two.getX(), three.getX()),
			average(one.getY(), two.getY(), three.getY()),
			average(one.getZ(), two.getZ(), three.getZ())
		);
	}

	private Rotation2d average(Rotation2d one, Rotation2d two, Rotation2d three) {
		return new Rotation2d(
			average(one.getRadians(), two.getRadians(), three.getRadians())
		);
	}
}
