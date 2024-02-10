package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.LoggedTunableNumber;

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
	public static final Mode currentMode = Mode.SIM;

	public static final ModuleType powerDistributionType = ModuleType.kRev;
	public static final boolean fieldOriented = false;
	public static final String logpath = "/media/sda1/";

	public static final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

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

		// Temp values for now
		public static final int kShooterTopPort = 7;
		public static final int kShooterBottomPort = 8;

		// Temp values for now
		public static final int kArm1Port = 9;
		public static final int kArm2Port = 10;

		public static final int kFrontLeftDriveMotorPort = 11;
		public static final int kFrontRightDriveMotorPort = 12;
		public static final int kBackLeftDriveMotorPort = 13;
		public static final int kBackRightDriveMotorPort = 14;

		public static final int kFrontLeftTurningMotorPort = 21;
		public static final int kFrontRightTurningMotorPort = 22;
		public static final int kBackLeftTurningMotorPort = 23;
		public static final int kBackRightTurningMotorPort = 24;
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

	public static final class ClimberConstants {
		public static final LoggedTunableNumber kClimbVoltage = new LoggedTunableNumber("Climber Voltage", 5);

		public static final double kClimberPositionReduction = 1.0;
		public static final double kClimberVelocityReduction = 1.0;
	}

	public static final class ArmConstants {
		public static final LoggedTunableNumber kIntakeVoltage = new LoggedTunableNumber("Intake Voltage", 7);

		public static final int kArmEncoderPort = 25;
		public static final I2C.Port kArmColorSensorPort = I2C.Port.kOnboard;

		// Color of note/lack of note + tolerance, subject to change.
		public static final Color kNoteColor = new Color(255, 127, 80);
		public static final double kColorConfidenceThreshold = 0.1;

		// Dummy values
		public static final LoggedTunableNumber kMaxSpeed = new LoggedTunableNumber("Max Speed", 3.5);
		public static final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber("Max Acceleration", 1);

		// Dummy Automatic Set Point values
		public static final LoggedTunableNumber kTaxiAngle = new LoggedTunableNumber("Taxi Angle", Math.PI / 4);
		public static final LoggedTunableNumber kRoombaAngle = new LoggedTunableNumber("Roomba Angle", Math.PI / 12);
		public static final LoggedTunableNumber kSpeakerAngle = new LoggedTunableNumber("Speaker Angle", 0.24434609527);
		public static final LoggedTunableNumber kAmpAngle = new LoggedTunableNumber("Amp Angle", Math.PI / 3); // temporary
		public static final LoggedTunableNumber kTrapAngle = new LoggedTunableNumber("Trap Angle", Math.PI / 5); // temporary

		public static final LoggedTunableNumber kP = new LoggedTunableNumber("[P]ID", 0);
		public static final LoggedTunableNumber kI = new LoggedTunableNumber("P[I]D", 0);
		public static final LoggedTunableNumber kD = new LoggedTunableNumber("PI[D]", 0);
		public static final LoggedTunableNumber kArmPIDTolerance = new LoggedTunableNumber("PID Tolerance", 0.5);

		// Another temp value
		public static final double kMaxArmAngle = 0;
		public static final double kMinArmAngle = 0;

		public static final double kIntakeReduction = 1.0;
		public static final double kArmReduction = 1.0; // CALCULATE
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
