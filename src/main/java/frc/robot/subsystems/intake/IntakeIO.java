package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
	@AutoLog
	class IntakeIOInputs {
		public double velocityRadPerSec = 0.0;
		public double appliedVoltage = 0.0;
		public double[] currentAmps = new double[]{};
		public double[] tempCelsius = new double[]{};
	}

	public default void updateInputs(IntakeIOInputs inputs) {
	}

	public default void setInputVoltage(double volts) {
	}

	public default void setBrakeMode(boolean brake) {
	}

	public default void stop() {
	}
}
