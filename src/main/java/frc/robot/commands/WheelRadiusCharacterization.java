// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class WheelRadiusCharacterization extends Command {
	private static final LoggedTunableNumber characterizationSpeed = new LoggedTunableNumber(
			"WheelRadiusCharacterization/SpeedRadsPerSec", 2.0);
	private static final double driveRadius = Math.hypot(DriveConstants.kTrackWidth / 2.0,
			DriveConstants.kWheelBase / 2.0);
	private static DoubleSupplier gyroYawRadsSupplier;

	public enum Direction {
		CLOCKWISE, COUNTER_CLOCKWISE;

		public int value() {
			switch (this) {
				case CLOCKWISE :
					return -1;
				case COUNTER_CLOCKWISE :
					return 1;
				default :
					return 1;
			}
		}
	}

	private final Swerve swerve;
	private final Odometry odometry;
	private final Direction omegaDirection;
	private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

	private double lastGyroYawRads = 0.0;
	private double accumGyroYawRads = 0.0;

	private double[] startWheelPositions;

	private double currentEffectiveWheelRadius = 0.0;

	public WheelRadiusCharacterization(Swerve swerve, Odometry odometry, Direction omegaDirection) {
		this.swerve = swerve;
		this.odometry = odometry;
		this.omegaDirection = omegaDirection;

		gyroYawRadsSupplier = () -> Math.IEEEremainder(this.odometry.getGyrometerHeading().getRadians(), 2 * Math.PI);

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		// Reset
		lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
		accumGyroYawRads = 0.0;

		startWheelPositions = swerve.getWheelRadiusCharacterizationPosition();

		omegaLimiter.reset(0);
	}

	@Override
	public void execute() {
		// Run drive at velocity
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0,
				omegaLimiter.calculate(omegaDirection.value() * characterizationSpeed.get()));
		swerve.setModuleStates(chassisSpeeds);

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

		swerve.stopModules();
	}
}
