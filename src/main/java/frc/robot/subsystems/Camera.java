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
	private List<Transform3d> poses;
	private PhotonPipelineResult result;

	public Camera(String name, Transform3d toRobot, AprilTagFieldLayout field) {
		camera = new PhotonCamera(name);
		poses = new ArrayList<Transform3d>();
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
				poses.add(target.getBestCameraToTarget());
				poses.add(target.getAlternateCameraToTarget());
			}
		}
	}

	public List<Transform3d> getPoses() {
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

	private List<Transform3d> orientToField(List<Transform3d> toTarget, int tag) {
		List<Transform3d> onField = new ArrayList<Transform3d>();

		for (Transform3d dist : toTarget) {
			Pose3d orField = field.getTagPose(tag).get().plus(dist.inverse()).plus(toRobot);
			onField.add(new Transform3d(orField.getX(), orField.getY(), orField.getX(), orField.getRotation()));
		}

		return onField;
	}
}
