package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

public class KMeans extends SubsystemBase {
	List<Transform3d> points;
	Transform3d k1, k2;
	List<Transform3d> cluster1 = new ArrayList<Transform3d>(), cluster2 = new ArrayList<Transform3d>();
	double diff;

	public KMeans() {
	}

	@Override
	public void periodic() {
	}

	// this is the method that should be called to find centroids
	public void updatePoints(List<Transform3d> points) {
		clean();

		for (int i = 1; i <= 80; i++) {
			for (Transform3d point : points) {
				reassign(point);
			}
		}
	}

	// this is the method that should be called to get centroids
	public Transform3d[] getCentroids() {
		return new Transform3d[] {k1, k2};
	}

	public List<List<Transform3d>> getClusters() {
		List<List<Transform3d>> toReturn = new ArrayList<List<Transform3d>>();
		toReturn.add(cluster1);
		toReturn.add(cluster2);

		return toReturn;
	}

	private Transform3d findCentroid(List<Transform3d> cluster) {
		if (cluster.size() == 1) {
			return cluster.get(0);
		} else {
			double x = 0;
			double y = 0;
			double z = 0;

			// rotational
			double roll = 0;
			double pitch = 0;
			double yaw = 0;

			for (Transform3d point : cluster) {
				x += point.getX();
				y += point.getY();
				z += point.getZ();

				// rotational
				roll += point.getRotation().getX();
				pitch += point.getRotation().getY();
				yaw += point.getRotation().getZ();
			}

			return new Transform3d(x / cluster.size(), y / cluster.size(), z / cluster.size(),
					new Rotation3d(roll / cluster.size(), pitch / cluster.size(), yaw / cluster.size()));
		}
	}

	private void reassign(Transform3d point) {
		remove(point);

		double to1 = euclideanDistance(point, k1) + rotationalDistance(point, k1);
		double to2 = euclideanDistance(point, k2) + rotationalDistance(point, k2);

		if (to1 < to2) {
			cluster1.add(point);
			k1 = findCentroid(cluster1);
		} else {
			cluster2.add(point);
			k2 = findCentroid(cluster2);
		}
	}

	// for distance
	private double euclideanDistance(Transform3d point, Transform3d centroid) {
		return Math.sqrt(Math.pow(Math.abs(point.getX() - centroid.getX()), 2) + Math.pow(Math.abs(point.getY() - centroid.getY()), 2)
				+ Math.pow(Math.abs(point.getZ() - centroid.getZ()), 2));
	}

	// for rotation
	private double rotationalDistance(Transform3d point, Transform3d centroid) {
		Rotation3d difference = centroid.getRotation().minus(point.getRotation());

		return Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2) + Math.pow(difference.getZ(), 2);
	}

	private void remove(Transform3d point) {
		cluster1.remove(point);
		cluster2.remove(point);
	}

	private void clean() {
		k1 = new Transform3d(Math.random() * 10, Math.random() * 10, Math.random() * 10, new Rotation3d());
		k2 = new Transform3d(Math.random() * 10, Math.random() * 10, Math.random() * 10, new Rotation3d());

		cluster1.clear();
		cluster2.clear();
	}
}
