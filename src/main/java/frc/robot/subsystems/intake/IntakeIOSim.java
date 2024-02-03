package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
	private final FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.01);
	private double appliedVolts = 0.0;

	public IntakeIOSim() {
		System.out.println("[Init] Creating IntakeIOSim");
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		intakeSim.update(0.02);
		inputs.velocityRadPerSec = intakeSim.getAngularVelocityRadPerSec();
		inputs.appliedVoltage = appliedVolts;
		inputs.currentAmps = new double[]{intakeSim.getCurrentDrawAmps()};
		inputs.tempCelsius = new double[]{};
	}

	@Override
	public void setInputVoltage(double volts) {
		appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		intakeSim.setInputVoltage(volts);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0);
	}
}
