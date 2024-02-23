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

	public Transform3d getDistance() {
		kmeans.updatePoints(getUnnestedList(right.getUnnestedData(), left.getUnnestedData(), back.getUnnestedData()));
		return kmeans.getCentroid();
	}

	public Rotation2d getRotToAlign() {
		kmeans.updatePoints(getUnnestedList(right.getSpeakerPoses(right.getTargets()),
				left.getSpeakerPoses(left.getTargets()), back.getSpeakerPoses(back.getTargets())));

		return kmeans.getCentroid().getRotation().toRotation2d();
	}

	public double getRadtoSpeaker() {
		return getRotToAlign().getRadians();
	}

	public double getShootVelocity() {
		kmeans.updatePoints(getUnnestedList(right.getSpeakerPoses(right.getTargets()),
				left.getSpeakerPoses(left.getTargets()), back.getSpeakerPoses(back.getTargets())));

		return kmeans.getCentroid().getTranslation()
				.getDistance(field.getTagPose(right.getSpeakerTag()).get().getTranslation()) / 1.0;
	}

	// thank you so much stephanie
	public double[] getShootRad() {
		Transform3d d = getDistance();
		double g = -9.80665;
		double x2 = Math.pow(d.getX(), 2);
		double thing = (g * x2) / 2 * Math.pow(getShootVelocity(), 2);

		double minus = Math.atan((d.getX()) + Math.sqrt(x2 - (4 * thing * (d.getY() + thing))) / (2 * thing));

		double plus = Math.atan((d.getX()) - Math.sqrt(x2 - (4 * thing * (d.getY() + thing))) / (2 * thing));

		return new double[]{plus, minus};
	}

	public void update() {
		right.update();
		left.update();
		back.update();
	}

	private List<Transform3d> getUnnestedList(List<Transform3d> nested1, List<Transform3d> nested2,
			List<Transform3d> nested3) {
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
}
