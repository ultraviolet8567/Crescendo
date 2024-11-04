package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;

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
