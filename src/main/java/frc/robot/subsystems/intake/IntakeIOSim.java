package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
	private final FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.01);

	private double appliedVoltage = 0.0;

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		intakeSim.update(0.02);
		inputs.velocityRadPerSec = intakeSim.getAngularVelocityRadPerSec();
		inputs.positionRads += intakeSim.getAngularVelocityRadPerSec() * 0.02;
		inputs.appliedVoltage = appliedVoltage;
		inputs.currentAmps = new double[] {intakeSim.getCurrentDrawAmps()};
	}

	@Override
	public void setInputVoltage(double volts) {
		appliedVoltage = volts;
		intakeSim.setInputVoltage(volts);
	}

	@Override
	public void stop() {
		appliedVoltage = 0.0;
		intakeSim.setInputVoltage(0.0);
	}
}
