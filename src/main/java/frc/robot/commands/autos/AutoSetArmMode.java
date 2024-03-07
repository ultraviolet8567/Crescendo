package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
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

		addRequirements(arm);
	}

	@Override
	public void initialize() {
		timer.start();
		arm.setArmMode(armMode);
	}

	@Override
	public boolean isFinished() {
		return (timer.get() - AutoConstants.kAutoArmTime) < 0.1;
	}
}
