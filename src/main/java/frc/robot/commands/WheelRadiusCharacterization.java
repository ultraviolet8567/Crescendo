// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Gyrometer;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class WheelRadiusCharacterization extends Command {
	private static final LoggedTunableNumber characterizationSpeed = new LoggedTunableNumber(
			"WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
	private static final double driveRadius = Math.hypot(DriveConstants.kTrackWidth / 2.0,
			DriveConstants.kWheelBase / 2.0);
	private static DoubleSupplier gyroYawRadsSupplier = () -> 0.0;

	@RequiredArgsConstructor
	public enum Direction {
		CLOCKWISE(-1), COUNTER_CLOCKWISE(1);

		private final int value;
	}

	private final Swerve swerve;
	private final Gyrometer odometry;
	private final Direction omegaDirection;
	private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

	private double lastGyroYawRads = 0.0;
	private double accumGyroYawRads = 0.0;

	private double[] startWheelPositions;

	private double currentEffectiveWheelRadius = 0.0;

	public WheelRadiusCharacterization(Swerve swerve, Gyrometer odometry, Direction omegaDirection) {
		this.swerve = swerve;
		this.odometry = odometry;
		this.omegaDirection = omegaDirection;
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		// Reset
		lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
		accumGyroYawRads = 0.0;

		startWheelPositions = swerve.getWheelRadiusCharacterizationPosition();
		gyroYawRadsSupplier = () -> (odometry.getHeading()).getRadians();

		omegaLimiter.reset(0);
	}

	@Override
	public void execute() {
		// Run drive at velocity
		swerve.runWheelRadiusCharacterization(
				omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()));

		// Get yaw and wheel positions
		accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
		lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
		double averageWheelPosition = 0.0;
		double[] wheelPositiions = swerve.getWheelRadiusCharacterizationPosition();
		for (int i = 0; i < 4; i++) {
			averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
		}
		averageWheelPosition /= 4.0;

		currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
		Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
		Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
		Logger.recordOutput("Drive/RadiusCharacterization/CurrentWheelRadiusInches",
				Units.metersToInches(currentEffectiveWheelRadius));
	}

	@Override
	public void end(boolean interrupted) {
		if (accumGyroYawRads <= Math.PI * 2.0) {
			System.out.println("Not enough data for characterization");
		} else {
			System.out.println(
					"Effective Wheel Radius: " + Units.metersToInches(currentEffectiveWheelRadius) + " inches");
		}
	}
}
