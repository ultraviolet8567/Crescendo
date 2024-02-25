package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {
	private PhotonCamera camera;
	private Transform3d toRobot;
	private AprilTagFieldLayout field;
	private List<List<Transform3d>> poses;
	private PhotonPipelineResult result;

	public Camera(String name, Transform3d toRobot, AprilTagFieldLayout field) {
		camera = new PhotonCamera(name);
		poses = new ArrayList<List<Transform3d>>();
		this.field = field;
		this.toRobot = toRobot;
	}

	public void update() {
		result = camera.getLatestResult();
		if (!poses.isEmpty()) {
			poses.clear();
		}

		if (result.hasTargets()) {
			List<PhotonTrackedTarget> targets = result.getTargets();

			for (PhotonTrackedTarget target : targets) {
				List<Transform3d> posePair = new ArrayList<Transform3d>();
				posePair.add(target.getBestCameraToTarget());
				posePair.add(target.getAlternateCameraToTarget());
				poses.add(posePair);
			}

			System.out.println(poses.get(0));
		}
	}

	public List<List<Transform3d>> getPoses() {
		System.out.println(poses);
		return poses;
	}

	public List<PhotonTrackedTarget> getTargets() {
		if (result.hasTargets()) {
			return result.getTargets();
		} else
			return null;
	}

	public List<Integer> getIDs() {
		if (result.hasTargets()) {
			List<Integer> ids = new ArrayList<Integer>();
			for (PhotonTrackedTarget target : result.targets) {
				ids.add(target.getFiducialId());
			}
			return ids;
		} else
			return null;
	}

	private List<Transform3d> getTagPoses(int tag) {
		if (result.hasTargets()) {
			List<PhotonTrackedTarget> targets = result.getTargets();
			List<Transform3d> list = new ArrayList<Transform3d>();

			for (PhotonTrackedTarget target : targets) {
				if (target.getFiducialId() == tag) {
					list.add(target.getBestCameraToTarget());
					list.add(target.getAlternateCameraToTarget());
				}
			}

			return list;
		} else
			return null;
	}

	public List<Transform3d> getSpeakerPoses(List<PhotonTrackedTarget> tags) {
		int speakerTag = getSpeakerTag();
		return orientToField(getTagPoses(speakerTag), speakerTag);
	}

	public List<Transform3d> getUnnestedData() {
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
