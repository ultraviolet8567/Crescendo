package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
	private PhotonCamera camera;
	private Transform3d toRobot;
	private AprilTagFieldLayout field;
	private List<List<Transform3d>> poses;
	private KMeans kmeans;

	public Camera(String name, Transform3d toRobot, KMeans kmeans, AprilTagFieldLayout field) {
		camera = new PhotonCamera(name);
		this.field = field;
		this.kmeans = kmeans;
		this.toRobot = toRobot;
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

	private List<Transform3d> getTagPoses(int tag) {
		var result = camera.getLatestResult();
		List<Transform3d> list = new ArrayList<Transform3d>();

		if (result.hasTargets()) {
			List<PhotonTrackedTarget> targets = result.getTargets();
			
			for (PhotonTrackedTarget target : targets) {
				if (target.getFiducialId() == tag) {
					list.add(target.getBestCameraToTarget());
					list.add(target.getAlternateCameraToTarget());
				}
			}

			return list;
		} else {
			return list;
		}
	}

	public Rotation2d getRotToAlign(List<PhotonTrackedTarget> tags) {
		int speakerTag = getSpeakerTag();
		List<Transform3d> toTag = orientToField(getTagPoses(speakerTag), speakerTag);

		kmeans.clean();
		kmeans.updatePoints(toTag);
		Transform3d distToTag = kmeans.getCentroid();

		return distToTag.getRotation().toRotation2d();
	}

	public double getShootVelocity() {
		double time = 1.0;
		int speakerTag = getSpeakerTag();
		List<Transform3d> toTag = orientToField(getTagPoses(speakerTag), speakerTag);

		kmeans.clean();
		kmeans.updatePoints(toTag);
		Translation3d robot = kmeans.getCentroid().getTranslation();

		return robot.getDistance(field.getTagPose(speakerTag).get().getTranslation()) / time;
	}

	public double getRadtoTag() {
		int speakerTag = getSpeakerTag();
		List<Transform3d> toTag = orientToField(getTagPoses(speakerTag), speakerTag);

		kmeans.clean();
		kmeans.updatePoints(toTag);
		Rotation3d rot = kmeans.getCentroid().getRotation();

		return rot.toRotation2d().getRadians();
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

	private Transform3d orientToField(Transform3d toTarget, int tag) {
		Pose3d onField = field.getTagPose(tag).get().plus(toTarget.inverse()).plus(toRobot);
		return new Transform3d(onField.getX(), onField.getY(), onField.getX(), onField.getRotation());
	}

	private List<Transform3d> orientToField(List<Transform3d> toTarget, int tag) {
		List<Transform3d> onField = new ArrayList<Transform3d>();

		for (Transform3d dist : toTarget) {
			onField.add(orientToField(dist, tag));
		}

		return onField;
	}
}
