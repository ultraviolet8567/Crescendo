package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;

public class ShooterIOSparkMax implements ShooterIO {
	public final CANSparkMax shooterTopMotor, shooterBottomMotor;
	public final RelativeEncoder shooterTopEncoder, shooterBottomEncoder;
	private final PIDController shooterTopPID, shooterBottomPID;
	private final SimpleMotorFeedforward shooterTopFF, shooterBottomFF;

	public ShooterIOSparkMax() {
		shooterTopMotor = new CANSparkMax(CAN.kShooterTopPort, MotorType.kBrushless);
		shooterTopMotor.enableVoltageCompensation(12.0);
		shooterTopMotor.setSmartCurrentLimit(40);
		shooterTopMotor.setIdleMode(IdleMode.kBrake);

		shooterBottomMotor = new CANSparkMax(CAN.kShooterBottomPort, MotorType.kBrushless);
		shooterBottomMotor.enableVoltageCompensation(12.0);
		shooterBottomMotor.setSmartCurrentLimit(40);
		shooterBottomMotor.setIdleMode(IdleMode.kBrake);

		shooterTopEncoder = shooterTopMotor.getEncoder();
		shooterTopEncoder.setPositionConversionFactor(1.0 / ArmConstants.kShooterReduction * 2 * Math.PI);
		shooterTopEncoder.setVelocityConversionFactor(1.0 / ArmConstants.kShooterReduction * 2 * Math.PI);

		shooterBottomEncoder = shooterBottomMotor.getEncoder();
		shooterBottomEncoder.setPositionConversionFactor(1.0 * 2 * Math.PI);
		shooterBottomEncoder.setVelocityConversionFactor(1.0 * 2 * Math.PI);

		shooterTopPID = new PIDController(ArmConstants.kShooterP.get(), ArmConstants.kShooterI.get(),
				ArmConstants.kShooterD.get());
		shooterBottomPID = new PIDController(ArmConstants.kShooterP.get(), ArmConstants.kShooterI.get(),
				ArmConstants.kShooterD.get());
		shooterTopFF = new SimpleMotorFeedforward(ArmConstants.kShooterS.get(), ArmConstants.kShooterV.get());
		shooterBottomFF = new SimpleMotorFeedforward(ArmConstants.kShooterS.get(), ArmConstants.kShooterV.get());
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.topVelocityRadPerSec = shooterTopEncoder.getVelocity();
		inputs.topAppliedVoltage = shooterTopMotor.getAppliedOutput() * shooterTopMotor.getBusVoltage();
		inputs.topCurrentAmps = shooterTopMotor.getOutputCurrent();

		inputs.bottomVelocityRadPerSec = shooterBottomEncoder.getVelocity();
		inputs.bottomAppliedVoltage = shooterBottomMotor.getAppliedOutput() * shooterBottomMotor.getBusVoltage();
		inputs.bottomCurrentAmps = shooterBottomMotor.getOutputCurrent();
	}

	// Sets the input voltage for the top motor/row of wheels
	@Override
	public void setTopInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		shooterTopMotor.setVoltage(appliedVolts);
	}

	// Sets the input voltage for the bottom motor/row of wheels
	@Override
	public void setBottomInputVoltage(double volts) {
		double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		shooterBottomMotor.setVoltage(appliedVolts);
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
		double topVolts = shooterTopPID.calculate(shooterTopEncoder.getVelocity(), topTargetVel)
				+ shooterTopFF.calculate(topTargetVel);
		double bottomVolts = shooterBottomPID.calculate(shooterBottomEncoder.getVelocity(), bottomTargetVel)
				+ shooterBottomFF.calculate(bottomTargetVel);

		setInputVoltage(topVolts, bottomVolts);
	}

	@Override
	public void stop() {
		setInputVoltage(0.0, 0.0);
	}

	public void setBrakeMode(boolean brake) {
		shooterTopMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
		shooterBottomMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
	}
}
