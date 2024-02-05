// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Test extends SubsystemBase {
	/** Creates a new Test. */
	public Test(KMeans kmeans) {
		KMeans name = new KMeans();
		ArrayList<Transform3d> pList = new ArrayList<>(Arrays.asList(new Transform3d(1, 2, 3, new Rotation3d()),
				new Transform3d(1.5, 2.5, 3.5, new Rotation3d()), new Transform3d(0.7, 1.8, 3.2, new Rotation3d()),
				new Transform3d(10, 10, 11, new Rotation3d()), new Transform3d(0, 0, 0, new Rotation3d()),
				new Transform3d(-1, -15, 30, new Rotation3d())));
		name.updatePoints(pList);

		Transform3d[] centroids = name.getCentroids();

		for (Transform3d centroid : centroids) {
			System.out.println(centroid);
		}

		for (List<Transform3d> list : name.getClusters()) {
			for (Transform3d point : list) {
				System.out.println(point);
			}
			System.out.println("list");
		}
	}

	@Override
	public void periodic() {

	}
}
