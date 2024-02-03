package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOSparkMax implements ClimberIO {
	public final CANSparkMax rightClimberMotor;
	public final RelativeEncoder rightClimberEncoder;
	public final CANSparkMax leftClimberMotor;
	public final RelativeEncoder leftClimberEncoder;

	public ClimberIOSparkMax() {
		System.out.println("[Init] Creating ClimberIOSparkMax");

		leftClimberMotor = new CANSparkMax(CAN.kLeftClimberPort, MotorType.kBrushless);
		leftClimberMotor.enableVoltageCompensation(12.0);
		leftClimberMotor.setSmartCurrentLimit(40);
		leftClimberMotor.setIdleMode(IdleMode.kBrake);

		leftClimberEncoder = leftClimberMotor.getEncoder();
		leftClimberEncoder.setPositionConversionFactor(1.0 / ClimberConstants.kClimberPositionReduction * 2 * Math.PI);
		leftClimberEncoder.setVelocityConversionFactor(1.0 / ClimberConstants.kClimberVelocityReduction * 2 * Math.PI);

		rightClimberMotor = new CANSparkMax(CAN.kRightClimberPort, MotorType.kBrushless);
		rightClimberMotor.enableVoltageCompensation(12.0);
		rightClimberMotor.setSmartCurrentLimit(40);
		rightClimberMotor.setIdleMode(IdleMode.kBrake);

		rightClimberEncoder = rightClimberMotor.getEncoder();
		rightClimberEncoder.setPositionConversionFactor(1.0 / ClimberConstants.kClimberPositionReduction * 2 * Math.PI);
		rightClimberEncoder.setVelocityConversionFactor(1.0 / ClimberConstants.kClimberVelocityReduction * 2 * Math.PI);
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.leftVelocityRadPerSec = leftClimberEncoder.getVelocity();
		inputs.leftPositionRads = leftClimberEncoder.getPosition();
		inputs.leftAppliedVoltage = leftClimberMotor.getAppliedOutput() * leftClimberMotor.getBusVoltage();
		inputs.leftCurrentAmps = new double[]{leftClimberMotor.getOutputCurrent()};
		inputs.leftTempCelsius = new double[]{leftClimberMotor.getMotorTemperature()};

		inputs.rightVelocityRadPerSec = rightClimberEncoder.getVelocity();
		inputs.rightPositionRads = rightClimberEncoder.getPosition();
		inputs.rightAppliedVoltage = rightClimberMotor.getAppliedOutput() * rightClimberMotor.getBusVoltage();
		inputs.rightCurrentAmps = new double[]{rightClimberMotor.getOutputCurrent()};
		inputs.rightTempCelsius = new double[]{rightClimberMotor.getMotorTemperature()};
	}

	public void setInputVoltage(double volts) {
		leftClimberMotor.setVoltage(volts);
		rightClimberMotor.setVoltage(volts);
	}

	public void setBrakeMode(boolean brake) {
		leftClimberMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
		rightClimberMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}

	public void stop() {
		setInputVoltage(0.0);
	}
}
