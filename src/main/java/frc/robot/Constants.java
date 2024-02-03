package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public final class Constants {
	/**
	 * General info Controller axis range: -1 to 1 Motor max: 5676 rot/min = 14.5
	 * ft/s = 4.5 m/s Speed cap: 5000 rot/min
	 *
	 * <p>
	 * Gyro: - Forward = ? - Left = ? - Counterclockwise = ?
	 *
	 * <p>
	 * Odometry - Forward = x+ - Left = y+ - Counterclockwise = z+
	 */
	public static final Mode currentMode = Mode.REAL;

	public static final ModuleType powerDistributionType = ModuleType.kRev;
	public static final boolean fieldOriented = true;
	public static final String logpath = "/media/sda1/";

	public static final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

	public static final AprilTagFields field = AprilTagFields.k2024Crescendo;
	public static final AprilTagFieldLayout fieldLayout = field.loadAprilTagLayoutField();

	public static final class OIConstants {
		public static final ControllerType controllerTypeDriver = ControllerType.XBOX;
		public static final ControllerType controllerTypeOperator = ControllerType.XBOX;

		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;

		public static final double kDeadband = 0.1;
	}

	public static final class CAN {
		// TODO: Update to proper wiring once motors are connected
		public static final int kArmPort = 1;
		public static final int kLeftClimberPort = 2;
		public static final int kRightClimberPort = 3;
		public static final int kIntakePort = 4;
		public static final int kLeftFlywheelPort = 5;
		public static final int kRightFlywheelPort = 6;

		public static final int kFrontLeftDriveMotorPort = 11;
		public static final int kFrontRightDriveMotorPort = 12;
		public static final int kBackLeftDriveMotorPort = 14;
		public static final int kBackRightDriveMotorPort = 13;

		public static final int kFrontLeftTurningMotorPort = 21;
		public static final int kFrontRightTurningMotorPort = 22;
		public static final int kBackLeftTurningMotorPort = 24;
		public static final int kBackRightTurningMotorPort = 23;
	}

	public static final class ModuleConstants {
		public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
		public static final double kDriveMotorGearRatio = 1 / 6.75;
		public static final double kTurningMotorGearRatio = 1 / (150 / 7.0);

		public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
		public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
		public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
		public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

		public static final double kPTurning = 0.5;
	}

	public static final class DriveConstants {
		// Distance between right and left wheels:
		public static final double kTrackWidth = Units.inchesToMeters(34); // TODO: Set to our dimensions
		// Distance between front and back wheels:
		public static final double kWheelBase = Units.inchesToMeters(32); // TODO: Set to our dimensions
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left (+/+)
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right (+/-)
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left (-/+)
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back right (-/-)

		// Edit depending on specs of MK4i
		public static final boolean kFrontLeftTurningEncoderReversed = true;
		public static final boolean kFrontRightTurningEncoderReversed = true;
		public static final boolean kBackLeftTurningEncoderReversed = true;
		public static final boolean kBackRightTurningEncoderReversed = true;

		public static final boolean kFrontLeftDriveEncoderReversed = true;
		public static final boolean kFrontRightDriveEncoderReversed = false;
		public static final boolean kBackLeftDriveEncoderReversed = true;
		public static final boolean kBackRightDriveEncoderReversed = false;

		public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
		public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
		public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
		public static final int kBackRightDriveAbsoluteEncoderPort = 3;

		public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
		public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
		public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
		public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

		public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(0.000); // TODO:
		// Determine
		// offset
		public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(0.000); // TODO:
		// Determine
		// offset
		public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(0.000); // TODO:
		// Determine
		// offset
		public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(0.000); // TODO:
		// Determine
		// offset

		public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
		public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;

		public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.9;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
				* 0.4;
		public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
		public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
	}

	public static enum Mode {
		// Running on a real robot
		REAL,
		// Running a simulator
		SIM,
		// In tuning mode
		TUNING,
		// Replaying from a log file
		REPLAY
	}

	public static enum ControllerType {
		XBOX, LOGITECH, JOYSTICK
	}
}
