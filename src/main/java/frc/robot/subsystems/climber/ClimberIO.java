package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	class ClimberIOInputs {
		public double leftVelocityRadPerSec = 0.0;
		public double leftPositionRads = 0.0;
		public double leftAppliedVoltage = 0.0;
		public double[] leftCurrentAmps = new double[]{};
		public double[] leftTempCelsius = new double[]{};

		public double rightVelocityRadPerSec = 0.0;
		public double rightPositionRads = 0.0;
		public double rightAppliedVoltage = 0.0;
		public double[] rightCurrentAmps = new double[]{};
		public double[] rightTempCelsius = new double[]{};
	}

	public default void updateInputs(ClimberIOInputs inputs) {
	}

	public default void setInputVoltage(double volts) {
	}

	public default void setBrakeMode(boolean brake) {
	}

	public default void stop() {
	}
}
