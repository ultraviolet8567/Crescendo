package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
	/*
	 * Declare components of subsystem here (motor controllers, encoders, sensors,
	 * etc.)
	 */
	private CANSparkMax climb;

	/*
	 * Initialize all components here, as well as any one-time logic to be completed
	 * on boot-up
	 */
	public Climber() {
		// ADD PORTS LATER GATERS

		climb = new CANSparkMax(1, MotorType.kBrushless);
		climb.setIdleMode(IdleMode.kBrake);
	}

	/* Runs periodically (about once every 20 ms) */
	@Override
	public void periodic() {
		
	}

	/* Define all subsystem-specific methods and enums here */
	public void startClimb(String direction) {
		if (direction == "extend"){
			climb.setInverted(false);
			climb.setVoltage(ClimberConstants.climbVoltage.get());
		}
		else {
			climb.setInverted(true);
			climb.setVoltage(ClimberConstants.climbVoltage.get());
		}
		/*
		if (positive) {
			climb.set(ClimberConstants.climbSpeed*((positive) ? 1 : -1));
		}
		else {
			climb.set(-ClimberConstants.climbSpeed);
		}
		*/
	}

	public void stopClimb() {
		climb.stopMotor();
	}
}
