package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
	@AutoLog
	class ArmIOInputs {
		public double velocityRadPerSec = 0.0;
		public double positionRads = 0.0;
		public double appliedVoltage = 0.0;
		public double currentAmps = 0.0;
	}

	default void updateInputs(ArmIOInputs inputs) {
	}

	default void initial() {
	}

	default void update() {
	}

	default void setInputVoltage(double volts) {
	}

	default void getPreset() {
	}

	default void stop() {
	}

	default void resetAbsoluteEncoders() {
	}

	default void resetPIDControllers() {
	}
}
