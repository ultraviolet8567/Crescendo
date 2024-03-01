/**
 * - field: positive theta is counter-clockwise, positive x-axis is away from alliance wall, positive y-axis is perpendicular & to the left of positive x
 * - robot: positive theta is counter-clockwise, positive x-axis is dir robot is facing, positive y-axis is perpendicular & to the left of robot
*/

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
	private Camera right, left, back;
	private KMeans kmeans;
	private AprilTagFieldLayout field = Constants.fieldLayout;

	public Vision(KMeans kmeans) {
		this.kmeans = kmeans;
		right = new Camera("frontRight", Constants.Cameras.frontRighttoRobot, field);
		left = new Camera("frontLeft", Constants.Cameras.frontLefttoRobot, field);
		back = new Camera("back", Constants.Cameras.backToRobot, field);
	}

	@Override
	public void periodic() {
		update();
	}

	// get robot pose
	public Transform3d getPose() {
		kmeans.updatePoints(getUnnestedList(right.getPoses(), left.getPoses(), back.getPoses()));
		return kmeans.getCentroid();
	}

	// get yaw to align to tag (rotation2d)
	public Rotation2d getRotToAlign() {
		kmeans.updatePoints(getUnnestedList(right.getPoses(), left.getPoses(), back.getPoses()));
		return kmeans.getCentroid().getRotation().toRotation2d();
	}

	// get value to align to tag (radians)
	public double getRadtoSpeaker() {
		return getRotToAlign().getRadians();
	}

	// get shoot velocity
	public double getShootVelocity() {
		kmeans.updatePoints(getUnnestedList(right.getPoses(), left.getPoses(), back.getPoses()));
		return kmeans.getCentroid().getTranslation()
				.getDistance(field.getTagPose(right.getSpeakerTag()).get().getTranslation()) / 1.0;
	}

	// thank you so much stephanie
	public double[] getShootRad() {
		Transform3d d = getPose();
		double g = -9.80665;
		double x2 = Math.pow(d.getX(), 2);
		double thing = (g * x2) / 2 * Math.pow(getShootVelocity(), 2);

		double minus = Math.atan((d.getX()) + Math.sqrt(x2 - (4 * thing * (d.getY() + thing))) / (2 * thing));
		double plus = Math.atan((d.getX()) - Math.sqrt(x2 - (4 * thing * (d.getY() + thing))) / (2 * thing));

		return new double[]{plus, minus};
	}

	// update all the cameras
	public void update() {
		right.update();
		left.update();
		back.update();
	}

	// unnest lists
	private List<Transform3d> getUnnestedList(List<Pose3d> nested1, List<Pose3d> nested2,
			List<Pose3d> nested3) {
		List<Transform3d> unnestedData = new ArrayList<Transform3d>();
		List<List<Pose3d>> nestedData = new ArrayList<List<Pose3d>>();
		nestedData.add(nested1);
		nestedData.add(nested2);
		nestedData.add(nested3);

		for (List<Pose3d> data : nestedData) {
			for (Pose3d point : data) {
				unnestedData.add(new Transform3d(point.getX(), point.getY(), point.getZ(), point.getRotation()));
			}
		}

		return unnestedData;
	}
}
