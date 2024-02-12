package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;

public class ShooterIOSparkMax implements ShooterIO {
	public final CANSparkMax shooterTopMotor, shooterBottomMotor;
	public final RelativeEncoder shooterTopEncoder, shooterBottomEncoder;
	private final Constraints shooterConstraints;
	private final ProfiledPIDController shooterPID;

	private double topAppliedVoltage = 1.0;
	private double bottomAppliedVoltage = 1.0;

	public ShooterIOSparkMax() {
		shooterTopMotor = new CANSparkMax(CAN.kShooterTopPort, MotorType.kBrushless);
		shooterTopMotor.enableVoltageCompensation(12.0);
		shooterTopMotor.setSmartCurrentLimit(40);
		shooterTopMotor.setIdleMode(IdleMode.kBrake);

		shooterBottomMotor = new CANSparkMax(CAN.kShooterBottomPort, MotorType.kBrushless);
		shooterBottomMotor.enableVoltageCompensation(12.0);
		shooterBottomMotor.setSmartCurrentLimit(40);
		shooterBottomMotor.setIdleMode(IdleMode.kBrake);

		shooterTopEncoder = shooterTopMotor.getEncoder();
		shooterTopEncoder.setPositionConversionFactor(1.0 * 2 * Math.PI);
		shooterTopEncoder.setVelocityConversionFactor(1.0 * 2 * Math.PI);

		shooterBottomEncoder = shooterBottomMotor.getEncoder();
		shooterBottomEncoder.setPositionConversionFactor(1.0 * 2 * Math.PI);
		shooterBottomEncoder.setVelocityConversionFactor(1.0 * 2 * Math.PI);

		shooterConstraints = new Constraints(ArmConstants.kShooterMaxSpeed.get(),
				ArmConstants.kShooterMaxAcceleration.get());
		shooterPID = new ProfiledPIDController(ArmConstants.kShooterP.get(), ArmConstants.kShooterI.get(),
				ArmConstants.kShooterD.get(), shooterConstraints);
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.topVelocityRadPerSec = shooterTopEncoder.getVelocity();
		inputs.topPositionRads = shooterTopEncoder.getPosition();
		inputs.topAppliedVoltage = shooterTopMotor.getAppliedOutput();
		inputs.topCurrentAmps = shooterTopMotor.getOutputCurrent();

		inputs.bottomVelocityRadPerSec = shooterBottomEncoder.getVelocity();
		inputs.bottomPositionRads = shooterBottomEncoder.getPosition();
		inputs.bottomAppliedVoltage = shooterBottomMotor.getAppliedOutput();
		inputs.bottomCurrentAmps = shooterBottomMotor.getOutputCurrent();
	}

	@Override
	public double getTopShooterVelocity() {
		return shooterTopEncoder.getVelocity();
	}

	@Override
	public double getBottomShooterVelocity() {
		return shooterBottomEncoder.getVelocity();
	}

	@Override
	public double calculateShooterTopVelocity(double topVel) {
		return (shooterPID.calculate(getTopShooterVelocity(), topVel));
	}

	@Override
	public double calculateShooterBottomVelocity(double bottomVel) {
		return (shooterPID.calculate(getBottomShooterVelocity(), bottomVel));
	}

	// Sets the input voltage for the top motor/row of wheels
	@Override
	public void setTopInputVoltage(double volts) {
		topAppliedVoltage = volts;
		shooterTopMotor.setVoltage(volts);
	}

	// Sets the input voltage for the bottom motor/row of wheels
	@Override
	public void setBottomInputVoltage(double volts) {
		bottomAppliedVoltage = volts;
		shooterBottomMotor.setVoltage(volts);
	}

	// Sets the input voltage for both motors/rows of wheels
	@Override
	public void setInputVoltage(double topVolts, double bottomVolts) {
		setTopInputVoltage(topVolts);
		setBottomInputVoltage(bottomVolts);
	}

	@Override
	public void stop() {
		topAppliedVoltage = 0.0;
		bottomAppliedVoltage = 0.0;
		setInputVoltage(0.0, 0.0);
	}
}
