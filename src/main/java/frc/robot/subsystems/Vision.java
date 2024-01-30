/**
 * things that we could do now:
 * horizontal alignment (swerve)
 * id (which side of the field we're on) affects calculations --> will differ based on field-based/robot-based
 * - field: positive theta is counter-clockwise, positive x-axis is away from alliance wall, positive y-axis is perpendicular & to the left of positive x
 * - robot: positive theta is counter-clockwise, positive x-axis is dir robot is facing, positive y-axis is perpendicular & to the left of robot
*/

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
	PhotonCamera camera = new PhotonCamera("OV9281");

	public Vision() {}

	@Override
	public void periodic() {
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			List<PhotonTrackedTarget> targets = result.getTargets();
			List<List<Transform3d>> poses = new ArrayList<List<Transform3d>>();

			for (PhotonTrackedTarget target : targets) {
				List<Transform3d> posePair = new ArrayList<Transform3d>();
				posePair.add(target.getBestCameraToTarget());
				posePair.add(target.getAlternateCameraToTarget());
				poses.add(posePair);
			}

			Transform3d meanPose = getMean(poses);
			// Transform3d bestSpace = leastWrongSpace(poses, meanPose);
			// Transform3d bestAngle = leastWrongAngle(poses, meanPose);
		}
	}

	public List<PhotonTrackedTarget> getTargets() {
		var result = camera.getLatestResult();

		if (result.hasTargets()) {
			return result.getTargets();
		}

		return new ArrayList<PhotonTrackedTarget>();
	}

	public Rotation2d getDistToAlign(boolean side, List<PhotonTrackedTarget> targets) {
		int idToFind = 0;

		if (!side) {
			idToFind = 4;
		} else {
			idToFind = 7;
		}

		PhotonTrackedTarget shootTarget = null;
		for (PhotonTrackedTarget target : targets) {
			if (target.getFiducialId() == idToFind) {
				shootTarget = target;
				break;
			}
		}

		double angleToTurn = 0;

		if (!Objects.isNull(shootTarget)) {
			angleToTurn = -1 * shootTarget.getYaw();
		}

		return new Rotation2d(angleToTurn);

	}

	public double distanceSquared(Transform3d pose) {
		return Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2);
	}

	// get mean of pose list
	public Transform3d getMean(List<List<Transform3d>> poses) {
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		double roll = 0.0;
		double pitch = 0.0;
		double yaw = 0.0;

		// return best value when only 1 target detected
		if (poses.size() == 1) {
			return poses.get(0).get(0);
		} else {
			for (List<Transform3d> posePair : poses) {
				x += posePair.get(0).getX() + posePair.get(1).getX();
				y += posePair.get(0).getY() + posePair.get(1).getY();
				z += posePair.get(0).getZ() + posePair.get(1).getZ();
				roll += posePair.get(0).getRotation().getX() + posePair.get(1).getRotation().getX();
				pitch += posePair.get(0).getRotation().getY() + posePair.get(1).getRotation().getY();
				yaw += posePair.get(0).getRotation().getZ() + posePair.get(1).getRotation().getZ();
			}

			Rotation3d angle = new Rotation3d(roll / (poses.size() * 2), pitch / (poses.size() * 2),
					yaw / (poses.size() * 2));
			return new Transform3d(x / (poses.size() * 2), y / (poses.size() * 2), z / (poses.size() * 2), angle);
		}
	}

	// find median of dataset
	public Transform3d getMedian(double[][] sorted) {
		return new Transform3d(findMedian(sorted[0]), findMedian(sorted[1]), findMedian(sorted[2]),
				new Rotation3d(findMedian(sorted[3]), findMedian(sorted[4]), findMedian(sorted[5])));
	}

	// find median of double array
	private double findMedian(double[] arr) {
		if (arr.length % 2 != 0) {
			return arr[arr.length - 1 / 2] + arr[arr.length / 2] / 2;
		} else {
			return arr[arr.length - 1 / 2];
		}
	}

	// sort dataset
	public double[][] sortList(List<List<Transform3d>> poses) {
		// distances
		double[] x = new double[poses.size() * 2], y = new double[poses.size() * 2], z = new double[poses.size() * 2];
		// angles
		double[] roll = new double[poses.size() * 2], pitch = new double[poses.size() * 2],
				yaw = new double[poses.size() * 2];

		int i = 0;
		for (List<Transform3d> posePair : poses) {
			// distances
			x[i] = posePair.get(0).getX();
			x[i + 1] = posePair.get(1).getX();
			y[i] = posePair.get(0).getY();
			y[i + 1] = posePair.get(1).getY();
			z[i] = posePair.get(0).getZ();
			z[i + 1] = posePair.get(1).getZ();

			// angles
			roll[i] = posePair.get(0).getRotation().getX();
			roll[i + 1] = posePair.get(1).getRotation().getX();
			pitch[i] = posePair.get(0).getRotation().getY();
			pitch[i + 1] = posePair.get(1).getRotation().getY();
			yaw[i] = posePair.get(0).getRotation().getZ();
			yaw[i + 1] = posePair.get(1).getRotation().getZ();
		}

		Arrays.parallelSort(x);
		Arrays.parallelSort(y);
		Arrays.parallelSort(z);
		Arrays.parallelSort(roll);
		Arrays.parallelSort(pitch);
		Arrays.parallelSort(yaw);

		return new double[][]{x, y, z, roll, pitch, yaw};
	}

	// get and sort distances
	public Transform3d leastWrongSpace(List<List<Transform3d>> poses, Transform3d mean) {
		Transform3d best = new Transform3d();
		double bestDist = 99999999;
		double dist;

		if (poses.size() == 1) {
			return poses.get(0).get(0);
		} else {
			for (List<Transform3d> posePair : poses) {
				dist = posePair.get(0).getTranslation().getDistance(mean.getTranslation());
				if (bestDist > dist) {
					best = posePair.get(0);
					bestDist = dist;
				}
				dist = posePair.get(1).getTranslation().getDistance(mean.getTranslation());
				if (bestDist > dist) {
					best = posePair.get(1);
					bestDist = dist;
				}
			}
			return best;
		}
	}

	// returns the least wrong transform3d angle-wise
	public Transform3d leastWrongAngle(List<List<Transform3d>> poses, Transform3d mean) {
		Transform3d leastWrong = poses.get(0).get(0);
		Rotation3d toCompare = mean.getRotation().minus(poses.get(0).get(0).getRotation());

		if (poses.size() == 1) {
			return leastWrong;
		} else {
			for (List<Transform3d> posePair : poses) {
				// difference between things & mean
				Rotation3d temp = mean.getRotation().minus(posePair.get(0).getRotation());
				Rotation3d altTemp = mean.getRotation().minus(posePair.get(1).getRotation());

				double compareSum = Math.pow(toCompare.getX(), 2) + Math.pow(toCompare.getY(), 2)
						+ Math.pow(toCompare.getZ(), 2);
				double tempSum = Math.pow(temp.getX(), 2) + Math.pow(temp.getY(), 2) + Math.pow(temp.getZ(), 2);
				double altSum = Math.pow(altTemp.getX(), 2) + Math.pow(altTemp.getY(), 2) + Math.pow(altTemp.getZ(), 2);

				if (tempSum >= altSum) {
					if (tempSum >= compareSum) {
						toCompare = temp;
						leastWrong = posePair.get(0);
					}
				} else {
					if (altSum >= compareSum) {
						toCompare = altTemp;
						leastWrong = posePair.get(1);
					}
				}
			}
			return leastWrong;
		}
	}
}
