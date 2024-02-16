package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmMode;
import frc.robot.subsystems.arm.Arm;

public class SetArmAngle extends Command {
	private Arm arm;
	private ArmMode armMode;

	public SetArmAngle(Arm arm, ArmMode armMode) {
		this.arm = arm;
		this.armMode = armMode;
	}

	@Override
	public void execute() {
		arm.setArmMode(armMode);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
