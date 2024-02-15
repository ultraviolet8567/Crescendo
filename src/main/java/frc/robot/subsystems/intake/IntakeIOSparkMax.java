package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
	public final CANSparkMax intakeMotor;
	public final RelativeEncoder intakeEncoder;

	public IntakeIOSparkMax() {
		System.out.println("[Init] Creating IntakeIOSparkMax");

		intakeMotor = new CANSparkMax(CAN.kIntakePort, MotorType.kBrushless);
		intakeMotor.enableVoltageCompensation(12.0);
		intakeMotor.setSmartCurrentLimit(40);
		intakeMotor.setIdleMode(IdleMode.kBrake);

		intakeEncoder = intakeMotor.getEncoder();
		intakeEncoder.setVelocityConversionFactor(1.0 / IntakeConstants.kIntakeReduction * 2 * Math.PI);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.velocityRadPerSec = intakeEncoder.getVelocity();
		inputs.appliedVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
		inputs.currentAmps = new double[]{intakeMotor.getOutputCurrent()};
		inputs.tempCelsius = new double[]{intakeMotor.getMotorTemperature()};
	}

	public void setInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		intakeMotor.setVoltage(appliedVolts);
	}

	public void stop() {
		setInputVoltage(0.0);
	}

	public void setBrakeMode(boolean brake) {
		intakeMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}
}
