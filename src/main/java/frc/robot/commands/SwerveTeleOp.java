package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

public class SwerveTeleOp extends Command {
	private final Swerve swerve;
	private final Odometry odometry;
	private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
	private final Supplier<Boolean> rotationOnFunction, rightBumper;
	private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

	// First supplier is the forward velocity, then its horizontal velocity, then
	// rotational velocity
	public SwerveTeleOp(Swerve swerve, Odometry odometry, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
			Supplier<Double> turningSpdFunction, Supplier<Boolean> rotationOnFunction, Supplier<Boolean> rightBumper) {
		this.swerve = swerve;
		this.odometry = odometry;
		this.xSpdFunction = xSpdFunction;
		this.ySpdFunction = ySpdFunction;
		this.turningSpdFunction = turningSpdFunction;
		this.rotationOnFunction = rotationOnFunction;
		this.rightBumper = rightBumper;
		this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
		this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
		this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		// Get real-time joystick inputs
		double xSpeed = xSpdFunction.get();
		double ySpeed = ySpdFunction.get();
		double turningSpeed = rotationOnFunction.get() ? turningSpdFunction.get() : 0;

		xSpeed *= (xSpeed > 0) ? (1.0 / 0.8) : (1.0 / 0.9);
		ySpeed *= (1.0 / 0.9);

		// Apply deadband (drops to 0 if joystick value is less than the deadband)
		if (Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)) < OIConstants.kDeadband) {
			xSpeed = 0;
			ySpeed = 0;
		}
		turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0;

		if (rightBumper.get()) {
			RobotContainer.getDriverJoystick().setRumble(RumbleType.kRightRumble, 0.025);
			xSpeed *= 0.33;
			ySpeed *= 0.33;
			turningSpeed *= 0.33;
		} else {
			RobotContainer.getDriverJoystick().setRumble(RumbleType.kBothRumble, 0);
		}

		xSpeed = MathUtil.clamp(xSpeed, -1, 1);
		ySpeed = MathUtil.clamp(ySpeed, -1, 1);
		turningSpeed = MathUtil.clamp(turningSpeed, -1, 1);

		// Make the driving smoother by using a slew rate limiter to minimize
		// acceleration
		// And scale joystick input to m/s or rad/sec
		xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		turningSpeed = turningLimiter.calculate(turningSpeed)
				* DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

		ChassisSpeeds chassisSpeeds;
		if (Constants.fieldOriented) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, odometry.getHeading());
		} else {
			chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
		}

		swerve.setModuleStates(chassisSpeeds);
	}

	@Override
	public void end(boolean interuppted) {
		swerve.stopModules();
	}
}
