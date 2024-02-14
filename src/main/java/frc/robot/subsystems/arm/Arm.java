package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Arm extends SubsystemBase {
	private final ArmIO io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
	public int armMode;

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
		Logger.recordOutput("ArmMode", armMode);
	}

	/* Define all subsystem-specific methods and enums here */
	public void setTargetAngle(double targetAngle) {
		// if (io.armWithinRange()) {
		io.setInputVoltage(io.calculateInputVoltage(targetAngle));
		// }
	}

	public void setTurnSpeed(double factor) {
		// if (io.armWithinRange()) {
		io.setInputVoltage(factor * ArmConstants.kMaxSpeed.get());
		// }
	}

	public void setArmMode(int mode) {
		armMode = mode;
	}

	public int getArmMode() {
		return armMode;
	}

	public Transform3d getDeltaY() {
		double y = ArmConstants.kArmLength * Math.sin(io.getPositionRads());
		return new Transform3d(0.0, y, 0.0, new Rotation3d());
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
