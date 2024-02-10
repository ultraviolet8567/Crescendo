package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
	private final FlywheelSim topShooterSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.1);
	private final FlywheelSim bottomShooterSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.1);

	private final Constraints shooterConstraints = new Constraints(ArmConstants.kShooterMaxSpeed.get(), ArmConstants.kShooterMaxAcceleration.get());;
	private final ProfiledPIDController shooterPID = new ProfiledPIDController(ArmConstants.kShooterP.get(), ArmConstants.kShooterI.get(), ArmConstants.kShooterD.get(),shooterConstraints);;

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
	public double getTopShooterVelocity() {
		return topShooterSim.getAngularVelocityRadPerSec();
	}

	@Override
	public double getBottomShooterVelocity() {
		return bottomShooterSim.getAngularVelocityRadPerSec();
	}

	@Override
	public double calculateShooterTopVelocity(double topVel) {
		return (shooterPID.calculate(getTopShooterVelocity(),topVel));
	}

	@Override
	public double calculateShooterBottomVelocity(double bottomVel) {
		return (shooterPID.calculate(getBottomShooterVelocity(),bottomVel));
	}

	// Sets the input voltage for the top motor/row of wheels
	@Override
	public void setTopInputVoltage(double volts) {
		topAppliedVoltage = volts;
		topShooterSim.setInputVoltage(volts);
	}

	// Sets the input voltage for the bottom motor/row of wheels
	@Override
	public void setBottomInputVoltage(double volts) {
		bottomAppliedVoltage = volts;
		bottomShooterSim.setInputVoltage(volts);
	}

	// Sets the input voltage for both motors/rows of wheels
	@Override
	public void setInputVoltage(double topVolts, double bottomVolts) {
		setTopInputVoltage(topVolts);
		setBottomInputVoltage(bottomVolts);
	}

	@Override
	public void stop() {
		topAppliedVoltage = 0.0;
		bottomAppliedVoltage = 0.0;
		setInputVoltage(0.0, 0.0);
	}
}
