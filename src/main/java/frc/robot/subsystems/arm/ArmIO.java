package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
	@AutoLog
	class ArmIOInputs {
		public double velocityRadPerSec = 0.0;
		public double positionRads = 0.0;
		public double appliedVoltage = 0.0;
		public double[] currentAmps = new double[]{};
		public double[] tempCelsius = new double[]{};
	}

	default void updateInputs(ArmIOInputs inputs) {
	}

	default void initial() {
	}

	default void update() {
	}

	default double getPositionRads() {
		return 0.0;
	}

	default double calculateInputVoltage(double setpoint) {
		return 0.0;
	}

	default void setInputVoltage(double volts) {
	}

	default void getPreset() {
	}

	default boolean armPastFrontLimit() {
		return true;
	}

	default boolean armPastBackLimit() {
		return true;
	}

	default boolean armWithinRange() {
		return false;
	}

	default void setBrakeMode() {
	}

	default void stop() {
	}

	default void resetAbsoluteEncoders() {
	}

	default void resetPIDControllers() {
	}
}
