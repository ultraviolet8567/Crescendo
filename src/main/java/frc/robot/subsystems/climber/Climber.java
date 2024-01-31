package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	private CANSparkMax climb;
	private double climbSpeed = 1000;
	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */

	public Climber() {
		climb = new CANSparkMax(1, MotorType.kBrushless);
		climb.setIdleMode(IdleMode.kBrake);
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {

	}
	/* Define all subsystem-specific methods and enums here */
	public void startClimb() {
		climb.set(climbSpeed);
	}

	public void stopClimb() {
		climb.stopMotor();
	}
}
