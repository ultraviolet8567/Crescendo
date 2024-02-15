// TODO: test with rotation
// currently at 3 ks -> to decrease to 2, delete every instance of "k3" and "cluster3"
// to increase, add ks and clusters

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

public class KMeans extends SubsystemBase {
	// variables
	private Transform3d k1, k2, k3;
	private List<Transform3d> cluster1 = new ArrayList<Transform3d>(), cluster2 = new ArrayList<Transform3d>(),
			cluster3 = new ArrayList<Transform3d>();

	// constructor (doesn't do anything)
	// TODO: change number of ks here?
	public KMeans() {
	}

	@Override
	public void periodic() {
	}

	// run kmeans clustering (test with number of iterations)
	public void updatePoints(List<Transform3d> points) {
		clean();

		for (int i = 1; i <= 80; i++) {
			for (Transform3d point : points) {
				reassign(point);
			}
		}
	}

	// return probably most accurate centroid
	public Transform3d getCentroid() {
		if (cluster1.size() > cluster2.size() && cluster2.size() > cluster3.size()) {
			return k1;
		} else if (cluster1.size() > cluster2.size() && cluster3.size() > cluster1.size()) {
			return k3;
		} else {
			return k2;
		}
	}

	// return centroids
	public Transform3d[] getCentroids() {
		return new Transform3d[]{k1, k2, k3};
	}

	// return clusters
	public List<List<Transform3d>> getClusters() {
		List<List<Transform3d>> toReturn = new ArrayList<List<Transform3d>>();
		toReturn.add(cluster1);
		toReturn.add(cluster2);
		toReturn.add(cluster3);

		return toReturn;
	}

	// find centroid (mean of each cluster)
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

	// compare distances, assign point to the cluster whose centroid it's closest to
	private void reassign(Transform3d point) {
		remove(point);

		double to1 = euclideanDistance(point, k1) + rotationalDistance(point, k1);
		double to2 = euclideanDistance(point, k2) + rotationalDistance(point, k2);
		double to3 = euclideanDistance(point, k3) + rotationalDistance(point, k3);

		if (to1 < to2 && to2 < to3) {
			cluster1.add(point);
			k1 = findCentroid(cluster1);
		} else if (to1 < to2 && to3 < to1) {
			cluster3.add(point);
			k3 = findCentroid(cluster3);
		} else {
			cluster2.add(point);
			k2 = findCentroid(cluster2);
		}
	}

	// find distance (x, y, z)
	private double euclideanDistance(Transform3d point, Transform3d centroid) {
		return Math.sqrt(Math.pow(Math.abs(point.getX() - centroid.getX()), 2)
				+ Math.pow(Math.abs(point.getY() - centroid.getY()), 2)
				+ Math.pow(Math.abs(point.getZ() - centroid.getZ()), 2));
	}

	// find distance (roll, pitch, yaw)
	private double rotationalDistance(Transform3d point, Transform3d centroid) {
		Rotation3d difference = centroid.getRotation().minus(point.getRotation());
		return Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2) + Math.pow(difference.getZ(), 2);
	}

	// remove given point from all clusters
	private void remove(Transform3d point) {
		cluster1.remove(point);
		cluster2.remove(point);
		cluster3.remove(point);
	}

	// reset kmeans
	public void clean() {
		k1 = new Transform3d(Math.random() * 10, Math.random() * 10, Math.random() * 10, new Rotation3d());
		k2 = new Transform3d(Math.random() * 10, Math.random() * 10, Math.random() * 10, new Rotation3d());
		k3 = new Transform3d(Math.random() * 10, Math.random() * 10, Math.random() * 10, new Rotation3d());

		cluster1.clear();
		cluster2.clear();
		cluster3.clear();
	}
}
