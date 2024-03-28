package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ClimberIOSim implements ClimberIO {
	private final FlywheelSim leftClimberSim;
	private final FlywheelSim rightClimberSim;
	private double appliedVolts = 0.0;

	public ClimberIOSim() {
		System.out.println("[Init] Creating ClimberIOSim");

		leftClimberSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.01);
		rightClimberSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.01);
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		leftClimberSim.update(0.02);
		rightClimberSim.update(0.02);
		inputs.leftVelocityRadPerSec = leftClimberSim.getAngularVelocityRadPerSec();
		inputs.rightVelocityRadPerSec = rightClimberSim.getAngularVelocityRadPerSec();
		inputs.leftAppliedVoltage = appliedVolts;
		inputs.rightAppliedVoltage = appliedVolts;
		inputs.leftCurrentAmps = new double[]{leftClimberSim.getCurrentDrawAmps()};
		inputs.rightCurrentAmps = new double[]{rightClimberSim.getCurrentDrawAmps()};
		inputs.leftTempCelsius = new double[]{};
		inputs.rightTempCelsius = new double[]{};
	}

	@Override
	public void setInputVoltage(double leftVolts, double rightVolts) {
		appliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
		leftClimberSim.setInputVoltage(appliedVolts);
		rightClimberSim.setInputVoltage(appliedVolts);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0, 0.0);
	}
}
