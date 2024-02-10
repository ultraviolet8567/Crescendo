package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;

public class SetArmAngle extends Command {
	private Arm arm;
	private double joystickY;
	private int armMode;

	public SetArmAngle(Arm arm, double joystickY, int armMode) {
		this.arm = arm;
		this.joystickY = joystickY;
		this.armMode = armMode;
	}

	@Override
	public void execute() {
		// The first thing that runs when command is called.
		if (armMode == 0) {
			manual();
		} else {
			automatic();
		}
	}

	public void manual() {
		if (Math.abs(joystickY) > OIConstants.kDeadband) {
			arm.setTurnSpeed(joystickY);
		}
	}

	public void automatic() {
		if (armMode == 1) {
			// Taxi Set Point
			arm.setTargetAngle(ArmConstants.kTaxiAngle.get());
		} else if (armMode == 2) {
			// Roomba Set Point
			arm.setTargetAngle(ArmConstants.kRoombaAngle.get());
		}
		// These need Vision
		else if (armMode == 3) {
			// Speaker Set Point
			arm.setTargetAngle(ArmConstants.kSpeakerAngle.get());
		} else if (armMode == 4) {
			// Amp Set Point
			arm.setTargetAngle(ArmConstants.kAmpAngle.get());
		} else if (armMode == 5) {
			// Trap Set Point
			arm.setTargetAngle(ArmConstants.kTrapAngle.get());
		}
	}

}
