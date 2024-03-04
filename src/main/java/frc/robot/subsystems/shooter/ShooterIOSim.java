package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.shooterTopGains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
	private final FlywheelSim topShooterSim, bottomShooterSim;

	private final PIDController shooterTopPID, shooterBottomPID;
	private SimpleMotorFeedforward shooterTopFF, shooterBottomFF;

	private double topAppliedVoltage, bottomAppliedVoltage;

	public ShooterIOSim() {
		System.out.println("[Init] Creating ShooterIOSim");

		topShooterSim = new FlywheelSim(DCMotor.getNEO(1), ShooterConstants.kShooterReduction, 0.1);
		bottomShooterSim = new FlywheelSim(DCMotor.getNEO(1), ShooterConstants.kShooterReduction, 0.1);
		shooterTopPID = new PIDController(shooterTopGains.kP(), shooterTopGains.kI(), shooterTopGains.kD());

		shooterBottomPID = new PIDController(shooterTopGains.kP(), shooterTopGains.kI(), shooterTopGains.kD());
		shooterTopFF = new SimpleMotorFeedforward(shooterTopGains.ffkS(), shooterTopGains.ffkV());
		shooterBottomFF = new SimpleMotorFeedforward(shooterTopGains.ffkS(), shooterTopGains.ffkV());

		topAppliedVoltage = 0.0;
		bottomAppliedVoltage = 0.0;
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		topShooterSim.update(0.02);
		bottomShooterSim.update(0.02);

		inputs.topVelocityRPM = topShooterSim.getAngularVelocityRPM();
		inputs.topAppliedVoltage = topAppliedVoltage;
		inputs.topCurrentAmps = new double[]{topShooterSim.getCurrentDrawAmps()};

		inputs.bottomVelocityRPM = bottomShooterSim.getAngularVelocityRPM();
		inputs.bottomAppliedVoltage = bottomAppliedVoltage;
		inputs.bottomCurrentAmps = new double[]{bottomShooterSim.getCurrentDrawAmps()};
	}

	@Override
	public void setTopInputVoltage(double volts) {
		topAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
		topShooterSim.setInputVoltage(volts);
	}

	@Override
	public void setBottomInputVoltage(double volts) {
		bottomAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
		bottomShooterSim.setInputVoltage(volts);
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

	@Override
	public void setVelocity(double topTargetVel, double bottomTargetVel) {
		double topVolts = shooterTopPID.calculate(topShooterSim.getAngularVelocityRPM(), topTargetVel)
				+ shooterTopFF.calculate(topTargetVel);
		double bottomVolts = shooterBottomPID.calculate(bottomShooterSim.getAngularVelocityRPM(), bottomTargetVel)
				+ shooterBottomFF.calculate(bottomTargetVel);

		setInputVoltage(topVolts, bottomVolts);
	}
}
