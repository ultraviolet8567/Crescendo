package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	CANSparkMax intake;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Intake() {
		intake = new CANSparkMax(CAN.kIntakePort, MotorType.kBrushless);
		intake.enableVoltageCompensation(12.0);
		intake.setSmartCurrentLimit(40);
		intake.setIdleMode(IdleMode.kBrake);
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		Logger.recordOutput("Setpoints/Intake", intake.get());
		Logger.recordOutput("Measured/Intake", intake.getEncoder().getVelocity());
	}

	/* Define all subsystem-specific methods and enums here */

	public void pickup() {
		intake.set(ArmConstants.intakeSpeed.get());
	}

	// Drop function if necessary
	// public void drop()
	// {
	// intake.set(-ArmConstants.intakeSpeed.get());
	// }

	public void stop() {
		intake.stopMotor();
	}
}
