package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {
	public final CANSparkFlex shooterTopMotor, shooterBottomMotor;
	public final RelativeEncoder shooterTopEncoder, shooterBottomEncoder;
	private final SparkPIDController shooterTopPID, shooterBottomPID;
	private SimpleMotorFeedforward shooterTopFF, shooterBottomFF;

	public ShooterIOSparkMax() {
		System.out.println("[Init] Creating ShooterIOSparkMax");

		shooterTopMotor = new CANSparkFlex(CAN.kShooterTopPort, MotorType.kBrushless);
		shooterTopMotor.enableVoltageCompensation(12.0);
		shooterTopMotor.setSmartCurrentLimit(40);
		shooterTopMotor.setIdleMode(IdleMode.kBrake);
		shooterTopMotor.setInverted(true);

		shooterBottomMotor = new CANSparkFlex(CAN.kShooterBottomPort, MotorType.kBrushless);
		shooterBottomMotor.enableVoltageCompensation(12.0);
		shooterBottomMotor.setSmartCurrentLimit(40);
		shooterBottomMotor.setIdleMode(IdleMode.kBrake);
		shooterBottomMotor.setInverted(true);

		shooterTopEncoder = shooterTopMotor.getEncoder();
		shooterTopEncoder.setVelocityConversionFactor(1.0 / ShooterConstants.kShooterReduction);

		shooterBottomEncoder = shooterBottomMotor.getEncoder();
		shooterBottomEncoder.setVelocityConversionFactor(1.0 / ShooterConstants.kShooterReduction);

		shooterTopPID = shooterTopMotor.getPIDController();
		shooterBottomPID = shooterBottomMotor.getPIDController();
		shooterTopFF = new SimpleMotorFeedforward(shooterGains.ffkS(), shooterGains.ffkV());
		shooterBottomFF = new SimpleMotorFeedforward(shooterGains.ffkS(), shooterGains.ffkV());

		setGains(shooterGains.kP(), shooterGains.kI(), shooterGains.kD(), shooterGains.ffkS(), shooterGains.ffkV());
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.topVelocityRPM = shooterTopEncoder.getVelocity();
		inputs.topAppliedVoltage = shooterTopMotor.getAppliedOutput() * shooterTopMotor.getBusVoltage();
		inputs.topCurrentAmps = shooterTopMotor.getOutputCurrent();
		inputs.topTempCelsius = new double[]{shooterBottomMotor.getMotorTemperature()};

		inputs.bottomVelocityRPM = shooterBottomEncoder.getVelocity();
		inputs.bottomAppliedVoltage = shooterBottomMotor.getAppliedOutput() * shooterBottomMotor.getBusVoltage();
		inputs.bottomCurrentAmps = shooterBottomMotor.getOutputCurrent();
		inputs.bottomTempCelsius = new double[]{shooterBottomMotor.getMotorTemperature()};
	}

	// Sets the input voltage for the top motor/row of wheels
	@Override
	public void setTopInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		shooterTopMotor.setVoltage(appliedVolts);
	}

	// Sets the input voltage for the bottom motor/row of wheels
	@Override
	public void setBottomInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		shooterBottomMotor.setVoltage(appliedVolts);
	}

	// Sets the input voltage for both motors/rows of wheels
	@Override
	public void setInputVoltage(double topVolts, double bottomVolts) {
		setTopInputVoltage(topVolts);
		setBottomInputVoltage(bottomVolts);
	}

	// Sets voltage to match the target velocities
	@Override
	public void setVelocity(double topTargetVel, double bottomTargetVel) {
		shooterTopPID.setReference(topTargetVel, CANSparkBase.ControlType.kVelocity, 0,
				shooterTopFF.calculate(topTargetVel));
		shooterBottomPID.setReference(bottomTargetVel, CANSparkBase.ControlType.kVelocity, 0,
				shooterBottomFF.calculate(bottomTargetVel));
	}

	@Override
	public void stop() {
		setInputVoltage(0.0, 0.0);
	}

	public void setGains(double kP, double kI, double kD, double ffkS, double ffkV) {
		shooterTopPID.setP(shooterGains.kP());
		shooterTopPID.setI(shooterGains.kI());
		shooterTopPID.setD(shooterGains.kD());
		shooterTopFF = new SimpleMotorFeedforward(ffkS, ffkV);

		shooterBottomPID.setP(shooterGains.kP());
		shooterBottomPID.setI(shooterGains.kI());
		shooterBottomPID.setD(shooterGains.kD());
		shooterBottomFF = new SimpleMotorFeedforward(ffkS, ffkV);
	}

	@Override
	public void setBrakeMode(boolean brake) {
		shooterTopMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
		shooterBottomMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}
}
