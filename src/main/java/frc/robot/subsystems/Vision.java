/**
 * things that we could do now:
 * horizontal alignment (swerve)
 * id (which side of the field we're on) affects calculations --> will differ based on field-based/robot-based
 * - field: positive theta is counter-clockwise, positive x-axis is away from alliance wall, positive y-axis is perpendicular & to the left of positive x
 * - robot: positive theta is counter-clockwise, positive x-axis is dir robot is facing, positive y-axis is perpendicular & to the left of robot
*/

// change detected values to field-oriented positions

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
	PhotonCamera camera = new PhotonCamera("OV9281");
	AprilTagFieldLayout field = Constants.fieldLayout;
	List<List<Transform3d>> poses;

	public Vision() {
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
		}
	}

	public List<List<Transform3d>> getPoses() {
		return poses;
	}

	public List<PhotonTrackedTarget> getTargets() {
		var result = camera.getLatestResult();

		if (result.hasTargets()) {
			return result.getTargets();
		}

		return new ArrayList<PhotonTrackedTarget>();
	}

	public List<Integer> getIDs() {
		List<Integer> ids = new ArrayList<Integer>();
		var result = camera.getLatestResult();

		if (result.hasTargets()) {
			for (PhotonTrackedTarget target : result.targets) {
				ids.add(target.getFiducialId());
			}
		}

		return ids;
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

	public List<Transform3d> orientToField(List<List<Transform3d>> poses, List<Integer> ids) {
		List<Transform3d> fieldOrientedPoses = new ArrayList<Transform3d>();
		int index = 0;

		for (List<Transform3d> pose : poses) {
			fieldOrientedPoses.add(orientTagToField(ids.get(index), pose.get(0)));
			fieldOrientedPoses.add(orientTagToField(ids.get(index), pose.get(1)));
			index++;
		}

		return fieldOrientedPoses;
	}

	public Transform3d orientTagToField(int tag, Transform3d distance) {
		return new Transform3d(field.getTagPose(tag).get().getX() + distance.getX(),
				field.getTagPose(tag).get().getY() + distance.getY(),
				field.getTagPose(tag).get().getZ() + distance.getZ(),
				field.getTagPose(tag).get().getRotation().plus(distance.getRotation()));
	}
}
