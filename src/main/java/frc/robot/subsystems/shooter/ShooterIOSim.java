package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ArmConstants;

public class ShooterIOSim implements ShooterIO {
	private final FlywheelSim topShooterSim = new FlywheelSim(DCMotor.getNEO(1), ArmConstants.kShooterReduction, 0.1);
	private final FlywheelSim bottomShooterSim = new FlywheelSim(DCMotor.getNEO(1), ArmConstants.kShooterReduction,
			0.1);

	private final PIDController shooterTopPID = new PIDController(ArmConstants.kShooterP.get(),
			ArmConstants.kShooterI.get(), ArmConstants.kShooterD.get());
	private final PIDController shooterBottomPID = new PIDController(ArmConstants.kShooterP.get(),
			ArmConstants.kShooterI.get(), ArmConstants.kShooterD.get());
	private final SimpleMotorFeedforward shooterTopFF = new SimpleMotorFeedforward(ArmConstants.kShooterS.get(),
			ArmConstants.kShooterV.get());
	private final SimpleMotorFeedforward shooterBottomFF = new SimpleMotorFeedforward(ArmConstants.kShooterS.get(),
			ArmConstants.kShooterV.get());

	private double topAppliedVoltage = 0.0;
	private double bottomAppliedVoltage = 0.0;

	public ShooterIOSim() {
		System.out.println("[Init] Creating ShooterIOSim");
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		topShooterSim.update(0.02);
		bottomShooterSim.update(0.02);

		inputs.topVelocityRPM = topShooterSim.getAngularVelocityRPM();
		inputs.topAppliedVoltage = topAppliedVoltage;
		inputs.topCurrentAmps = topShooterSim.getCurrentDrawAmps();

		inputs.bottomVelocityRPM = bottomShooterSim.getAngularVelocityRPM();
		inputs.bottomAppliedVoltage = bottomAppliedVoltage;
		inputs.bottomCurrentAmps = bottomShooterSim.getCurrentDrawAmps();
	}

	// Sets the input voltage for the top motor/row of wheels
	@Override
	public void setTopInputVoltage(double volts) {
		topAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
		topShooterSim.setInputVoltage(volts);
	}

	// Sets the input voltage for the bottom motor/row of wheels
	@Override
	public void setBottomInputVoltage(double volts) {
		bottomAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
		bottomShooterSim.setInputVoltage(volts);
	}

	// Sets the input voltage for both motors/rows of wheels
	@Override
	public void setInputVoltage(double topVolts, double bottomVolts) {
		setTopInputVoltage(topVolts);
		setBottomInputVoltage(bottomVolts);
	}

	// Sets voltage to match the target velocities
	@Override
	public void setVelocity(double topTargetVel, double bottomTargetVel) {
		double topVolts = shooterTopPID.calculate(topShooterSim.getAngularVelocityRPM(), topTargetVel)
				+ shooterTopFF.calculate(topTargetVel);
		double bottomVolts = shooterBottomPID.calculate(bottomShooterSim.getAngularVelocityRPM(), bottomTargetVel)
				+ shooterBottomFF.calculate(bottomTargetVel);

		setInputVoltage(topVolts, bottomVolts);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0, 0.0);
	}
}
