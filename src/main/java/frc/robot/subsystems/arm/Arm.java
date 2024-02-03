package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	public final CANSparkMax arm1, arm2;
	private final Constraints armConstraints;
	private final ProfiledPIDController armPID;

	// For use in presets (yet to be made)
	private final DutyCycleEncoder armEncoder;

	public double targetAngle;

	private final ArmIO io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Arm(ArmIO io) {
		this.io = io;

		arm1 = new CANSparkMax(CAN.kArm1Port, MotorType.kBrushless);
		arm1.enableVoltageCompensation(12.0);
		arm1.setSmartCurrentLimit(40);
		arm1.setIdleMode(IdleMode.kBrake);

		arm2 = new CANSparkMax(CAN.kArm2Port, MotorType.kBrushless);
		arm2.enableVoltageCompensation(12.0);
		arm2.setSmartCurrentLimit(40);
		arm2.setIdleMode(IdleMode.kBrake);

		// For use in presets (yet to be made)
		armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderPort);

		armConstraints = new Constraints(ArmConstants.kMaxSpeed.get(), ArmConstants.kMaxAcceleration.get());

		armPID = new ProfiledPIDController(ArmConstants.kP.get(), ArmConstants.kI.get(), ArmConstants.kD.get(),
				armConstraints);
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		io.update();

		Logger.processInputs("Arms", inputs);

		Logger.recordOutput("Setpoints/Intake", arm1.get());
		// Logger.recordOutput("Measured/Intake", arm1.getEncoder().getVelocity());

		// Logger.recordOutput("Setpoints/Intake", arm2.get());
		// Logger.recordOutput("Measured/Intake", arm2.getEncoder().getVelocity());

		io.setInputVoltage(0.1);
	}

	/* Define all subsystem-specific methods and enums here */
	public void setTargetAngle(double targetAngle) {
		double armAngle = armEncoder.getAbsolutePosition();

		// Both motors spin the same and in the same direction
		arm1.set(armPID.calculate(armAngle, targetAngle));
		arm2.set(armPID.calculate(armAngle, targetAngle));
	}

	public void setTurnSpeed(double factor) {
		if (arm1.getEncoder().getPosition() + factor * ArmConstants.kMaxSpeed.get() > ArmConstants.kMinArmAngle
				&& arm1.getEncoder().getPosition()
						+ factor * ArmConstants.kMaxSpeed.get() < ArmConstants.kMaxArmAngle) {
		}
		arm1.set(factor * ArmConstants.kMaxSpeed.get());
		arm2.set(factor * ArmConstants.kMaxSpeed.get());
	}
}
