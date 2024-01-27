package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
	private final FlywheelSim topShooterSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.01);
	private final FlywheelSim bottomShooterSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.01);

	private double topAppliedVoltage = 0.0;
	private double bottomAppliedVoltage = 0.0;

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		topShooterSim.update(0.02);
		bottomShooterSim.update(0.02);

		inputs.topVelocityRadPerSec = topShooterSim.getAngularVelocityRadPerSec();
		inputs.topPositionRads += topShooterSim.getAngularVelocityRadPerSec() * 0.02;
		inputs.topAppliedVoltage = topAppliedVoltage;
		inputs.topCurrentAmps = topShooterSim.getCurrentDrawAmps();

		inputs.bottomVelocityRadPerSec = bottomShooterSim.getAngularVelocityRadPerSec();
		inputs.bottomPositionRads += bottomShooterSim.getAngularVelocityRadPerSec() * 0.02;
		inputs.bottomAppliedVoltage = bottomAppliedVoltage;
		inputs.bottomCurrentAmps = bottomShooterSim.getCurrentDrawAmps();
	}

	@Override
	public void setTopInputVoltage(double volts) {
		topAppliedVoltage = volts;
		topShooterSim.setInputVoltage(volts);
	}

	@Override
	public void setBottomInputVoltage(double volts) {
		bottomAppliedVoltage = volts;
		bottomShooterSim.setInputVoltage(volts);
	}

	/*
	 * @Override public void stop() { topAppliedVoltage = 0.0; bottomAppliedVoltage
	 * = 0.0; topShooterSim.setTopInputVoltage(0.0);
	 * bottomShooterSim.setBottomInputVoltage(0.0); }
	 */
}
