package frc.robot.subsystems.shooter;

import static frc.robot.Constants.GainsConstants.*;

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

		shooterBottomPID = new PIDController(shooterBottomGains.kP(), shooterBottomGains.kI(), shooterBottomGains.kD());
		shooterTopFF = new SimpleMotorFeedforward(shooterTopGains.ffkS(), shooterTopGains.ffkV());
		shooterBottomFF = new SimpleMotorFeedforward(shooterBottomGains.ffkS(), shooterBottomGains.ffkV());

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

	@Override
	public void setGains(double tkP, double tkI, double tkD, double tffkS, double tffkV, double bkP, double bkI,
			double bkD, double bffkS, double bffkV) {
		shooterTopPID.setPID(tkP, tkI, tkD);
		shooterTopFF = new SimpleMotorFeedforward(tffkS, tffkV);

		shooterBottomPID.setPID(bkP, bkI, bkD);
		shooterBottomFF = new SimpleMotorFeedforward(bffkS, bffkV);
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
