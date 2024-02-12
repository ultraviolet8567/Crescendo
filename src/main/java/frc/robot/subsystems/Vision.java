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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
	private PhotonCamera camera = new PhotonCamera("OV9281");
	private AprilTagFieldLayout field = Constants.fieldLayout;
	private List<List<Transform3d>> poses;
	private KMeans kmeans;

	public Vision(KMeans kmeans) {
		this.kmeans = kmeans;
	}

	@Override
	public void periodic() {
		update();
	}

	public void update() {
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

	public Rotation2d getRotToAlign(List<PhotonTrackedTarget> tags) {
		int speakerTag = getSpeakerTag();

		List<Transform3d> toTag = new ArrayList<Transform3d>();

		for (PhotonTrackedTarget tag : tags) {
			toTag.add(orientTagToField(speakerTag, tag.getBestCameraToTarget()));
			toTag.add(orientTagToField(speakerTag, tag.getAlternateCameraToTarget()));
		}

		kmeans.clean();
		kmeans.updatePoints(toTag);
		Transform3d distToTag = kmeans.getCentroid();

		return distToTag.getRotation().toRotation2d();
	}

	public double getShootVelocity(List<PhotonTrackedTarget> tags) {
		double time = 1.0;
		int speakerTag = getSpeakerTag();

		List<Transform3d> toTag = new ArrayList<Transform3d>();

		for (PhotonTrackedTarget tag : tags) {
			toTag.add(orientTagToField(speakerTag, tag.getBestCameraToTarget()));
			toTag.add(orientTagToField(speakerTag, tag.getAlternateCameraToTarget()));
		}

		kmeans.clean();
		kmeans.updatePoints(toTag);
		Translation3d robot = kmeans.getCentroid().getTranslation();


		return robot.getDistance(field.getTagPose(speakerTag).get().getTranslation()) / time;
	}

	public Transform3d getDistance() {
		kmeans.updatePoints(unNestList());
		return kmeans.getCentroid();
	}

	private List<Transform3d> unNestList() {
		List<Transform3d> unnested = new ArrayList<Transform3d>();

		for (List<Transform3d> posePair : poses) {
			unnested.add(posePair.get(0));
			unnested.add(posePair.get(1));
		}

		return unnested;
	}

	public int getSpeakerTag() {
		Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		int speakerTag = 0;

		if (alliance.isPresent()) {
			if (alliance.get() == DriverStation.Alliance.Red) {
				speakerTag = 4;
			} else {
				speakerTag = 7;
			}
		}

		return speakerTag;
	}

	private List<Transform3d> orientToField(List<List<Transform3d>> poses, int tag) {
		List<Transform3d> fieldOrientedPoses = new ArrayList<Transform3d>();

		for (List<Transform3d> pose : poses) {
			fieldOrientedPoses.add(orientTagToField(tag, pose.get(0)));
			fieldOrientedPoses.add(orientTagToField(tag, pose.get(1)));
		}

		return fieldOrientedPoses;
	}

	private Transform3d orientTagToField(int tag, Transform3d distance) {
		return new Transform3d(field.getTagPose(tag).get().getX() + distance.getX(),
				field.getTagPose(tag).get().getY() + distance.getY(),
				field.getTagPose(tag).get().getZ() + distance.getZ(),
				field.getTagPose(tag).get().getRotation().plus(distance.getRotation()));
	}
}
