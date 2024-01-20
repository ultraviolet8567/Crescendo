// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
	PhotonCamera camera = new PhotonCamera("OV9281");
	/** Creates a new Vision. */
	public Vision() {

	}

	@Override
	public void periodic() {
		var result = camera.getLatestResult();
		boolean hasTargets = result.hasTargets();
		if (hasTargets) {
			System.out.println(result.getBestTarget().getFiducialId());
			/*
			 * List<PhotonTrackedTarget> targets = result.getTargets();
			 *
			 * List<List<Transform3d>> poses; for (PhotonTrackedTarget target : targets) {
			 * List<Transform3d> posePair; posePair.add(target.getBestCameraToTarget());
			 * posePair.add(target.getAlternateCameraToTarget()); poses.add(posePair); }
			 * //TODO: check for combination of poses that are the most similar (lowest std
			 * dev for each combination?)
			 */
		}
		// This method will be called once per scheduler run
	}
}
