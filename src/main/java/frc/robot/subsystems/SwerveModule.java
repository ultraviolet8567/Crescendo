package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turningEncoder;

	private final PIDController turningPidController;

	public SwerveModulePosition modulePosition;

	private final AnalogInput absoluteEncoder;
	private final double absoluteEncoderOffset;
	private final boolean absoluteEncoderReversed;

	public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
			int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
		this.absoluteEncoderOffset = absoluteEncoderOffset;
		this.absoluteEncoderReversed = absoluteEncoderReversed;
		absoluteEncoder = new AnalogInput(absoluteEncoderID);

		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		driveMotor.enableVoltageCompensation(12.0);
		driveMotor.setSmartCurrentLimit(40);
		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted(driveMotorReversed);

		turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
		turningMotor.enableVoltageCompensation(12.0);
		turningMotor.setSmartCurrentLimit(40);
		turningMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setInverted(turningMotorReversed);

		driveEncoder = driveMotor.getEncoder();
		turningEncoder = turningMotor.getEncoder();

		driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
		driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
		turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
		turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

		turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
		turningPidController.enableContinuousInput(-Math.PI, Math.PI);

		resetEncoders();
	}

	public double getDrivePosition() {
		return driveEncoder.getPosition();
	}

	public double getTurningPosition() {
		return turningEncoder.getPosition();
	}

	public double getDriveVelocity() {
		return driveEncoder.getVelocity();
	}

	public double getTurningVelocity() {
		return turningEncoder.getVelocity();
	}

	public double getAbsoluteEncoderAngle() {
		double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
		angle *= 2 * Math.PI;
		angle += absoluteEncoderOffset;
		angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);

		return angle * (absoluteEncoderReversed ? -1 : 1);
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
		turningEncoder.setPosition(getAbsoluteEncoderAngle());
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
	}

	public void setDesiredState(SwerveModuleState state) {
		if (Math.abs(state.speedMetersPerSecond) < 0.001) {
			stop();
		} else {
			state = SwerveModuleState.optimize(state, getState().angle);
			driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
			turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
		}
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDrivePosition(), getState().angle);
	}

	public void stop() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}
}
