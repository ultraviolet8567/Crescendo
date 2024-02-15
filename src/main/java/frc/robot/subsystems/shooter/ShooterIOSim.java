package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
	private final FlywheelSim topShooterSim = new FlywheelSim(DCMotor.getNEO(1), ShooterConstants.kShooterReduction,
			0.1);
	private final FlywheelSim bottomShooterSim = new FlywheelSim(DCMotor.getNEO(1), ShooterConstants.kShooterReduction,
			0.1);

	private final PIDController shooterTopPID = new PIDController(shooterGains.kP(), shooterGains.kI(),
			shooterGains.kD());
	private final PIDController shooterBottomPID = new PIDController(shooterGains.kP(), shooterGains.kI(),
			shooterGains.kD());
	private SimpleMotorFeedforward shooterTopFF = new SimpleMotorFeedforward(shooterGains.ffkS(), shooterGains.ffkV());
	private SimpleMotorFeedforward shooterBottomFF = new SimpleMotorFeedforward(shooterGains.ffkS(),
			shooterGains.ffkV());

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

	public void setGains(double kP, double kI, double kD, double ffkS, double ffkV) {
		shooterTopPID.setPID(kP, kI, kD);
		shooterTopFF = new SimpleMotorFeedforward(ffkS, ffkV);

		shooterBottomPID.setPID(kP, kI, kD);
		shooterBottomFF = new SimpleMotorFeedforward(ffkS, ffkV);
	}
}
