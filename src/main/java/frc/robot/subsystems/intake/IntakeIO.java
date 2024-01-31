package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
	@AutoLog
	class IntakeIOInputs {
		public double velocityRadPerSec = 0.0;
		public double positionRads = 0.0;
		public double appliedVoltage = 0.0;
		public double currentAmps = 0.0;
	}

	default void updateInputs(IntakeIOInputs inputs) {
	}

	default void setInputVoltage(double volts) {
	}

	default void stop() {
	}
}
