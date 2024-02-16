package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmMode;
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
		if (arm.armMode == ArmMode.MANUAL || Math.abs(rightJoystickSupplier.get()) > OIConstants.kDeadband) {
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

		arm.setArmMode(ArmMode.MANUAL);
	}

	public void automatic() {
		switch (arm.armMode) {
			case ROOMBA :
				arm.setTargetAngle(ArmConstants.kRoombaAngle);
			case SPEAKER :
				arm.setTargetAngle(ArmConstants.kSpeakerAngle);
			case AMP :
				arm.setTargetAngle(ArmConstants.kAmpAngle);
			case TRAP :
				arm.setTargetAngle(ArmConstants.kTrapAngle);
			default :
				arm.setTargetAngle(ArmConstants.kTaxiAngle);
		}
	}

}
