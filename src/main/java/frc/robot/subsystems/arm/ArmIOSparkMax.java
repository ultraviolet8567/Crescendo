package frc.robot.subsystems.arm;

import static frc.robot.Constants.GainsConstants.armGains;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;
import frc.robot.util.SparkConfig;
import frc.robot.util.SparkConfig.SparkType;
import org.littletonrobotics.junction.Logger;

public class ArmIOSparkMax implements ArmIO {
	public final CANSparkMax arm1Motor, arm2Motor;
	public final RelativeEncoder arm1Encoder, arm2Encoder;
	private final DutyCycleEncoder armEncoder;
	private final ProfiledPIDController armPID;
	private ArmFeedforward armFF;

	private final Mechanism2d arm;
	private final MechanismRoot2d armRoot;
	private final MechanismLigament2d superstructureMech, armMech;

	public ArmIOSparkMax() {
		System.out.println("[Init] Creating ArmIOSparkMax");

		arm1Motor = new CANSparkMax(CAN.kArm1Port, MotorType.kBrushless);
		SparkConfig.config(arm1Motor, SparkType.kSparkMax);

		arm2Motor = new CANSparkMax(CAN.kArm2Port, MotorType.kBrushless);
		SparkConfig.config(arm1Motor, SparkType.kSparkMax);

		arm1Encoder = arm1Motor.getEncoder();
		arm1Encoder.setVelocityConversionFactor(1.0 / ArmConstants.kArmReduction * 2 * Math.PI);

		arm2Encoder = arm2Motor.getEncoder();
		arm2Encoder.setVelocityConversionFactor(1.0 / ArmConstants.kArmReduction * 2 * Math.PI);

		armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort);

		armPID = new ProfiledPIDController(armGains.kP(), armGains.kI(), armGains.kD(),
				new Constraints(ArmConstants.kMaxSpeed.get(), ArmConstants.kMaxAcceleration.get()));
		armFF = new ArmFeedforward(armGains.ffkS(), armGains.ffkG(), armGains.ffkV(), armGains.ffkA());

		// Mechanism visualizer
		arm = new Mechanism2d(2, 2);
		armRoot = arm.getRoot("Arm Root", 1, 0.5);
		superstructureMech = armRoot.append(new MechanismLigament2d("Superstructure", 0.2, 90));
		armMech = superstructureMech.append(new MechanismLigament2d("Arm", 0.58, 90));
		armMech.append(new MechanismLigament2d("Intake", 0.36, 64.0));
		armMech.append(new MechanismLigament2d("Shooter", 0.11, 244.0));
	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		inputs.velocityRadPerSec = arm1Encoder.getVelocity();
		inputs.appliedVoltage = arm1Motor.getAppliedOutput() * arm1Motor.getBusVoltage();
		inputs.positionRads = getPositionRads();
		inputs.currentAmps = new double[]{arm1Motor.getOutputCurrent()};
		inputs.tempCelsius = new double[]{arm1Motor.getMotorTemperature()};
		inputs.withinRange = armWithinRange();

		// Update mechanism visualizer
		armMech.setAngle(getPositionRads());
		Logger.recordOutput("Arm/Mechanism", arm);
	}

	@Override
	public void setInputVoltage(double volts) {
		double appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);

		// Both motors spin the same and in the same direction
		arm1Motor.setVoltage(appliedVoltage);
		arm2Motor.setVoltage(appliedVoltage);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0);
	}

	@Override
	public void setGains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
		armPID.setPID(kP, kI, kD);
		armFF = new ArmFeedforward(ffkS, ffkG, ffkV, ffkA);
	}

	@Override
	public void setBrakeMode(boolean brake) {
		arm1Motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
		arm2Motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public double getPositionRads() {
		double position = armEncoder.getAbsolutePosition();
		position *= 2 * Math.PI;
		position += ArmConstants.kArmEncoderOffset;
		return position * (ArmConstants.kArmEncoderReversed ? -1 : 1);
	}

	@Override
	public void setPosition(double setpoint) {
		double volts = armPID.calculate(getPositionRads(), setpoint) + armFF.calculate(getPositionRads(), setpoint);
		setInputVoltage(volts);
	}

	@Override
	public boolean armPastFrontLimit() {
		return getPositionRads() < ArmConstants.kMinArmAngle;
	}

	@Override
	public boolean armPastBackLimit() {
		return getPositionRads() > ArmConstants.kMaxArmAngle;
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
