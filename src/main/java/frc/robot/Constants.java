package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

	public static final Mode currentMode = Mode.REAL;
	public static final RobotType currentRobot = (currentMode == Mode.SIM) ? RobotType.SIMBOT : RobotType.REALBOT;

	public static final ModuleType powerDistributionType = ModuleType.kRev;
	public static final boolean fieldOriented = true;
	public static final String logpath = "/media/sda1/";

	public static final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

	public static final boolean lightsExist = false;

	// Color of note and detection tolerance, subject to change.
	public static final Color kNoteColor = new Color(138, 94, 23);
	public static final double kColorConfidenceThreshold = 0.8;

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
		public static final double kTrackWidth = Units.inchesToMeters(21);
		// Distance between front and back wheels:
		public static final double kWheelBase = Units.inchesToMeters(21);
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

		public static final boolean kFrontLeftDriveEncoderReversed = false;
		public static final boolean kFrontRightDriveEncoderReversed = true;
		public static final boolean kBackLeftDriveEncoderReversed = false;
		public static final boolean kBackRightDriveEncoderReversed = true;

		public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
		public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
		public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
		public static final int kBackRightDriveAbsoluteEncoderPort = 3;

		public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
		public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
		public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
		public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

		public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -1.677;
		public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.804;
		public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.452;
		public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.132;

		public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
		public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;

		public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.9;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
				* 0.4;
		public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
		public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

		public static final HolonomicPathFollowerConfig kHolonomicConfig = new HolonomicPathFollowerConfig(
				new PIDConstants(0.25, 0.0, 0.0), // Translation PID constants
				new PIDConstants(0.25, 0.0, 0.0), // Rotation PID constants
				kTeleDriveMaxSpeedMetersPerSecond, // Max module speed, in m/s
				Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2, // Drive base radius in meters.
																					// Distance from robot center to
																					// furthest module.
				new ReplanningConfig() // Default path replanning config. See the API for the options here
		);
	}

	public static final class ClimberConstants {
		public static final LoggedTunableNumber kClimbVoltage = new LoggedTunableNumber("Climber Voltage", 5);

		public static final double kClimberPositionReduction = 1.0;
		public static final double kClimberVelocityReduction = 1.0;
	}

	public static final class ArmConstants {
		public static final int kArmEncoderPort = 0;
		public static final double kArmEncoderOffset = -2.65016;
		public static final boolean kArmEncoderReversed = true;

		// Physics
		public static final double kArmLength = 0.58;
		public static final double kArmReduction = 144.0;
		public static final double kArmJKgMetersSquared = 0.515;

		// Constraints
		public static final double kMaxArmAngle = 0.0;
		public static final double kMinArmAngle = -1.76625;

		public static final LoggedTunableNumber kMaxSpeed = new LoggedTunableNumber("Arm/Max Speed", 1);
		public static final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber("Arm/Max Acceleration", 1);

		public static final LoggedTunableNumber kManualVoltage = new LoggedTunableNumber("Arm/ManualVoltage", 8);

		// Arm presets
		public static final double kTaxiAngle = -1.175;
		public static final double kRoombaAngle = -0.04396;
		public static final double kSpeakerAngle = -0.13816;
		public static final double kAmpAngle = -1.620;
		public static final double kTrapAngle = -0.13816;

		// Control
		public static final LoggedTunableNumber kArmPIDTolerance = new LoggedTunableNumber("Arm/PID Tolerance", 0.5);
	}

	public static final class ShooterConstants {
		public static final double kShooterReduction = 1.0;

		public static final LoggedTunableNumber kShooterPIDTolerance = new LoggedTunableNumber("Shooter/PID Tolerance",
				0.5);

		public static final LoggedTunableNumber kAmpRPM = new LoggedTunableNumber("Shooter/Amp RPM", 500);
		public static final LoggedTunableNumber kSpeakerRPM = new LoggedTunableNumber("Shooter/Speaker RPM", 6300);
		public static final LoggedTunableNumber kTrapRPM = new LoggedTunableNumber("Shooter/Manual RMM", 1000);
		public static final LoggedTunableNumber kIdleRPM = new LoggedTunableNumber("Shooter/Idle RPM", 500);
	}

	public static final class IntakeConstants {
		public static final double kIntakeReduction = 1.0;

		public static final LoggedTunableNumber kIntakeVoltage = new LoggedTunableNumber("Intake/Voltage", 10);
	}

	public static final class GainsConstants {
		public static final Gains shooterGains = switch (currentRobot) {
			case REALBOT -> new Gains(0.0006, 0.0, 0.05, 0.33329, 0.00083, 0.0, 0.0);
			case SIMBOT -> new Gains(1.0, 0.0, 0.0, 0.009078, 0.00103, 0.0, 0.0);
		};
		public static final Gains armGains = switch (currentRobot) {
			case REALBOT -> new Gains(1.0, 0.0, 0.0, 0.009078, 2.77, 0.06, 1.07);
			case SIMBOT -> new Gains(1.0, 0.0, 0.0, 0.009078, 2.77, 0.06, 1.07);
		};
	}

	public record Gains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
	}

	public static enum RobotType {
		/** Physical robot */
		REALBOT,
		/** Simulated robot */
		SIMBOT
	}

	public static enum Mode {
		/** Running on a real robot */
		REAL,
		/** Running a simulator */
		SIM,
		/** In tuning mode */
		TUNING,
		/** Replaying from a log file */
		REPLAY
	}

	public static enum ControllerType {
		XBOX, LOGITECH, JOYSTICK
	}

}
