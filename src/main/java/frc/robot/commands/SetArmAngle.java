package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class SetArmAngle extends Command {
	private Arm arm;
	private int armMode;

	public SetArmAngle(Arm arm, int armMode) {
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
