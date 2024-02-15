package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;
import java.util.function.Supplier;

public class MoveArm extends Command {
	private Arm arm;
	private Supplier<Double> rightJoystickSupplier;

	public MoveArm(Arm arm, Supplier<Double> rightJoystickSupplier) {
		this.arm = arm;
		this.rightJoystickSupplier = rightJoystickSupplier;

		addRequirements(arm);
	}

	@Override
	public void execute() {
		// The first thing that runs when command is called.
		if (arm.armMode == 0 || Math.abs(rightJoystickSupplier.get()) > OIConstants.kDeadband) {
			manual();
		} else {
			automatic();
		}
	}

	public void manual() {
		if (Math.abs(rightJoystickSupplier.get()) > OIConstants.kDeadband) {
			arm.setTurnSpeed(rightJoystickSupplier.get());
		} else {
			arm.setTurnSpeed(0.0);
		}

		arm.setArmMode(0);
	}

	public void automatic() {
		if (arm.armMode == 1) {
			// Taxi Set Point
			arm.setTargetAngle(ArmConstants.kTaxiAngle);
		} else if (arm.armMode == 2) {
			// Roomba Set Point
			arm.setTargetAngle(ArmConstants.kRoombaAngle);
		}
		// These need Vision
		else if (arm.armMode == 3) {
			// Speaker Set Point
			arm.setTargetAngle(ArmConstants.kSpeakerAngle);
		} else if (arm.armMode == 4) {
			// Amp Set Point
			arm.setTargetAngle(ArmConstants.kAmpAngle);
		} else if (arm.armMode == 5) {
			// Trap Set Point
			arm.setTargetAngle(ArmConstants.kTrapAngle);
		} else {
			arm.setTargetAngle(ArmConstants.kTaxiAngle);
		}
	}

}
