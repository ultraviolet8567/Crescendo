package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
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
		if (arm.getArmMode() == ArmMode.MANUAL || Math.abs(rightJoystickSupplier.get()) > OIConstants.kDeadband) {
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
		arm.setPosition(arm.getPresetAngle());
	}

}
