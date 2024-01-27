package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.CAN;

public class IntakeIOSparkMax implements IntakeIO {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	public final CANSparkMax intakeMotor;
    public final RelativeEncoder intakeEncoder;

    private double appliedVoltage = 0.0;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public IntakeIOSparkMax() {
		intakeMotor = new CANSparkMax(CAN.kIntakePort, MotorType.kBrushless);
		intakeMotor.enableVoltageCompensation(12.0);
		intakeMotor.setSmartCurrentLimit(40);
		intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPositionConversionFactor(1.0 /*/ reduction*/ * 2 * Math.PI);
        intakeEncoder.setVelocityConversionFactor(1.0 /*/ reduction*/ * 2 * Math.PI);    
	}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadPerSec = intakeEncoder.getVelocity();
        inputs.positionRads = intakeEncoder.getPosition();
        inputs.appliedVoltage = intakeMotor.getAppliedOutput();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
    }

    public void setInputVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    public void stop() {
        appliedVoltage = 0.0;
        setInputVoltage(appliedVoltage);
    }
}
