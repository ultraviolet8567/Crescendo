/**
 * things that we could do now:
 * horizontal alignment (swerve)
 * id (which side of the field we're on) affects calculations --> will differ based on field-based/robot-based
 * - field: positive theta is counter-clockwise, positive x-axis is away from alliance wall, positive y-axis is perpendicular & to the left of positive x
 * - robot: positive theta is counter-clockwise, positive x-axis is dir robot is facing, positive y-axis is perpendicular & to the left of robot
*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
	private Camera right, left, back;
	private KMeans kmeans;
	private Gyrometer gyro;
	private AprilTagFieldLayout field = Constants.fieldLayout;
	private double yaw_allowance = 0.5;
	private double adjustment_allowance = 0.5;

	// TODO: fill in name & find the transform3ds
	public Vision(KMeans kmeans, Gyrometer gyro) {
		this.kmeans = kmeans;
		this.gyro = gyro;
		right = new Camera("", Constants.Cameras.frontRighttoRobot, field);
		left = new Camera("", Constants.Cameras.frontLefttoRobot, field);
		back = new Camera("", Constants.Cameras.backToRobot, field);
	}

	@Override
	public void periodic() {
		update();
	}

	public Transform3d getDistance() {
		kmeans.updatePoints(getGrandData());
		return kmeans.getCentroid();
	}

	public Rotation2d getRotToAlign() {
		kmeans.updatePoints(getUnnestedList(
			right.getSpeakerPoses(right.getTargets()), 
			left.getSpeakerPoses(left.getTargets()), 
			back.getSpeakerPoses(back.getTargets()))
		);
		
		return kmeans.getCentroid().getRotation().toRotation2d();
	}
	
	public double getRadtoSpeaker() {
		return getRotToAlign().getRadians();
	}

	public double getShootVelocity() {
		kmeans.updatePoints(getUnnestedList(
			right.getSpeakerPoses(right.getTargets()), 
			left.getSpeakerPoses(left.getTargets()), 
			back.getSpeakerPoses(back.getTargets()))
		);

		return kmeans.getCentroid().getTranslation().getDistance(field.getTagPose(right.getSpeakerTag()).get().getTranslation()) / 1.0;
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

	private List<Transform3d> getUnnestedList(List<Transform3d> nested1, List<Transform3d> nested2, List<Transform3d> nested3) {
		List<Transform3d> unnestedData = new ArrayList<Transform3d>();
		List<List<Transform3d>> nestedData = new ArrayList<List<Transform3d>>();
		nestedData.add(nested1);
		nestedData.add(nested2);
		nestedData.add(nested3);

		for (List<Transform3d> data : nestedData) {
			for (Transform3d point : data) {
				unnestedData.add(point);
			}
		}
		
		return unnestedData;
	}

	private List<Transform3d> getGrandData() {
		List<Transform3d> bruh = getUnnestedList(right.getUnnestedData(), left.getUnnestedData(), back.getUnnestedData());
		return adjust(bruh, gyro.getHeading().getRadians());
	}

	private List<Transform3d> adjust(List<Transform3d> initialList, double yaw) { //if this breaks, convert everything from transform3d
		List<Transform3d> outList = new ArrayList<Transform3d>();
		Transform3d current;
		if (yaw < yaw_allowance || yaw > 2 * Math.PI - yaw_allowance) {
			for (Transform3d transform : initialList) {
				current = transform;
				if (current.getRotation().getZ() > 2 * Math.PI - adjustment_allowance)
					current = new Transform3d(current.getTranslation(), 
					new Rotation3d(current.getRotation().getX(), 
					current.getRotation().getY(), 
					current.getRotation().getZ() - 2 * Math.PI)); 
				outList.add(current);
			}
		}
		else {
			outList = initialList;
		}
		return outList;
	}
}
