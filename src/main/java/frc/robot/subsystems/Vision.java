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
import java.util.Map;
import java.util.Objects;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
	PhotonCamera camera = new PhotonCamera("OV9281");

	public Vision() {
		GenericEntry alliance = Shuffleboard.getTab("Horrible, horrible Shuffleboard").add("Alliance", false)
				.withWidget("Toggle Button")
				.withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"))
				.getEntry();
	}

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
			Transform3d bestSpace = leastWrongSpace(poses, meanPose);
			Transform3d bestAngle = leastWrongAngle(poses, meanPose);

			String side = findFieldSide(targets);
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

	// return side of field we're on
	// true if on blue, false if on red
	public String findFieldSide(List<PhotonTrackedTarget> targets) {
		PhotonTrackedTarget closest = targets.get(0);

		for (PhotonTrackedTarget target : targets) {
			Transform3d pose = target.getBestCameraToTarget();
			if (distanceSquared(closest.getBestCameraToTarget()) > distanceSquared(pose)) {
				closest = target;
			}
		}

		int id = closest.getFiducialId();
		if (id <= 2 || id >= 14 || (id >= 6 && id <= 8)) {
			return "Red";
		}

		return "Blue";
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

				double meanSum = Math.pow(mean.getX(), 2) + Math.pow(mean.getY(), 2) + Math.pow(mean.getZ(), 2);
				double tempSum = Math.pow(temp.getX(), 2) + Math.pow(temp.getY(), 2) + Math.pow(temp.getZ(), 2);
				double altSum = Math.pow(altTemp.getX(), 2) + Math.pow(altTemp.getY(), 2) + Math.pow(altTemp.getZ(), 2);

				if (tempSum >= altSum) {
					if (tempSum >= meanSum) {
						toCompare = temp;
						leastWrong = posePair.get(0);
					}
				} else {
					if (altSum >= meanSum) {
						toCompare = altTemp;
						leastWrong = posePair.get(1);
					}
				}
			}
			return leastWrong;
		}
	}
}
