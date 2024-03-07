package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;

public class AutoSetArmMode extends Command {
	private Timer timer;
	private ArmMode armMode;
	private Arm arm;

	public AutoSetArmMode(Arm arm, ArmMode armMode) {
		this.arm = arm;
		this.armMode = armMode;
		timer = new Timer();
	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
		arm.setArmMode(armMode);

		System.out.println("Change arm mode");
	}

	@Override
	public boolean isFinished() {
		return arm.atSetpoint();
	}
}
