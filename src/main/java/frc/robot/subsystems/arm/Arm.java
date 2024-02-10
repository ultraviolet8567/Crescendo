package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
	private final ArmIO io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Arm(ArmIO io) {
		this.io = io;
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		io.update();

		Logger.processInputs("Arms", inputs);
	}

	/* Define all subsystem-specific methods and enums here */
	public void setTargetAngle(double targetAngle) {
		if (io.armWithinRange()) {
			io.setInputVoltage(io.calculateInputVoltage(targetAngle));
		}
	}

	public void setTurnSpeed(double factor) {
		if (io.getPositionRads() + factor * ArmConstants.kMaxSpeed.get() > ArmConstants.kMinArmAngle
				&& io.getPositionRads() + factor * ArmConstants.kMaxSpeed.get() < ArmConstants.kMaxArmAngle) {
			io.setInputVoltage(factor * ArmConstants.kMaxSpeed.get());
		}
	}

	public void stop() {
		io.stop();
	}

	public void resetAbsoluteEncoders() {
		io.resetAbsoluteEncoders();
	}

	public void resetPIDControllers() {
		io.resetPIDControllers();
	}
}
