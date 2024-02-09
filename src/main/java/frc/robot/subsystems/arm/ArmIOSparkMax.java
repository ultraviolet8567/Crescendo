package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class ArmIOSparkMax implements ArmIO {
	public final CANSparkMax arm1Motor, arm2Motor;
    public final RelativeEncoder arm1Encoder, arm2Encoder;
	private final Constraints armConstraints;
	private final ProfiledPIDController armPID;
    private final DutyCycleEncoder armEncoder;
	public double targetAngle;

	public ArmIOSparkMax() {
		System.out.println("[Init] Creating SparkIOSparkMax");

		arm1Motor = new CANSparkMax(CAN.kArm1Port, MotorType.kBrushless);
		arm1Motor.enableVoltageCompensation(12.0);
		arm1Motor.setSmartCurrentLimit(40);
		arm1Motor.setIdleMode(IdleMode.kBrake);

        arm1Encoder = arm1Motor.getEncoder();
		arm1Encoder.setVelocityConversionFactor(1.0 / Constants.ArmConstants.kArmReduction * 2 * Math.PI);

		arm2Motor = new CANSparkMax(CAN.kArm2Port, MotorType.kBrushless);
		arm2Motor.enableVoltageCompensation(12.0);
		arm2Motor.setSmartCurrentLimit(40);
		arm2Motor.setIdleMode(IdleMode.kBrake);

        arm2Encoder = arm2Motor.getEncoder();
		arm2Encoder.setVelocityConversionFactor(1.0 / Constants.ArmConstants.kArmReduction * 2 * Math.PI);

        armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort);

		armConstraints = new Constraints(ArmConstants.kMaxSpeed.get(), ArmConstants.kMaxAcceleration.get());

		armPID = new ProfiledPIDController(ArmConstants.kP.get(), ArmConstants.kI.get(), ArmConstants.kD.get(),
				armConstraints);
	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		inputs.velocityRadPerSec = arm1Encoder.getVelocity();
		inputs.appliedVoltage = arm1Motor.getAppliedOutput() * arm1Motor.getBusVoltage();
        inputs.positionRads = armEncoder.get();
		inputs.currentAmps = new double[]{arm1Motor.getOutputCurrent()};
		inputs.tempCelsius = new double[]{arm1Motor.getMotorTemperature()};
	}

    @Override
    public double getPositionRads() {
        return armEncoder.get();
    }

    @Override
    public double calculateInputVoltage(double targetAngle) {
		// Both motors spin the same and in the same direction
		return (armPID.calculate(armEncoder.getAbsolutePosition(), targetAngle) * 12.0);
	}

    @Override
	public void setInputVoltage(double volts) {
		arm1Motor.setVoltage(volts);
        arm2Motor.setVoltage(volts);
	}
   
	public void setBrakeMode(boolean brake) {
		arm1Motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
		arm2Motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}

    @Override
	public void stop() {
		setInputVoltage(0.0);
	}

    @Override
	public void resetAbsoluteEncoders() {
        armEncoder.reset();
	}

	@Override
	public void resetPIDControllers() {
		armPID.reset(getPositionRads());
	}
}
