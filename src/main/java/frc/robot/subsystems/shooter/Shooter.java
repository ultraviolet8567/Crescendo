package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	CANSparkMax shooterTop;
	CANSparkMax shooterBottom;
	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */

	// Add shooter ports!!!!!

	public Shooter() {
		shooterTop = new CANSparkMax(CAN.kShooterTopPort, MotorType.kBrushless);
		shooterTop.enableVoltageCompensation(12.0);
		shooterTop.setSmartCurrentLimit(40);
		shooterTop.setIdleMode(IdleMode.kBrake);

		shooterBottom = new CANSparkMax(CAN.kShooterBottomPort, MotorType.kBrushless);
		shooterBottom.enableVoltageCompensation(12.0);
		shooterBottom.setSmartCurrentLimit(40);
		shooterBottom.setIdleMode(IdleMode.kBrake);

	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		Logger.recordOutput("Setpoints/Intake", shooterTop.get());
		Logger.recordOutput("Measured/Intake", shooterTop.getEncoder().getVelocity());
		Logger.recordOutput("Setpoints/Intake", shooterBottom.get());
		Logger.recordOutput("Measured/Intake", shooterBottom.getEncoder().getVelocity());
	}

	/* Define all subsystem-specific methods and enums here */
	public void shoot(double topVel, double bottomVel) {
		shooterTop.set(topVel);
		shooterBottom.set(bottomVel);
	}

	public void stop() {
		shooterTop.stopMotor();
		shooterBottom.stopMotor();
	}
}
