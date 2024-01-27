package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.Pickup;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ControllerIO;

/**
 * This class is where the bulk of the robot should be declared. The structure
 * of the robot (including subsystems, commands, and trigger mappings) should be
 * declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Arm arm;
	private final Climber climber;
	private final Intake intake;
	private final Odometry odometry;
	private final Shooter shooter;
	private final Swerve swerve;

	// Joysticks
	private static final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
	private static final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

	public RobotContainer() {
		switch (Constants.currentMode) {
			case REAL -> {
				arm = new Arm();
				climber = new Climber();
				intake = new Intake(new IntakeIOSparkMax());
				odometry = new Odometry();
				shooter = new Shooter();
				swerve = new Swerve();
			}
			case SIM -> {
				arm = new Arm();
				climber = new Climber();
				intake = new Intake(new IntakeIOSim());
				odometry = new Odometry();
				shooter = new Shooter();
				swerve = new Swerve();
			}
			default -> {
				arm = new Arm();
				climber = new Climber();
				intake = new Intake(new IntakeIO() {
				});
				odometry = new Odometry();
				shooter = new Shooter();
				swerve = new Swerve();
			}
		}
		// Configure default commands for driving and arm movement
		swerve.setDefaultCommand(new SwerveTeleOp(swerve, odometry,
				() -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getLeftY()),
				() -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getLeftX()),
				() -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
				() -> OIConstants.controllerTypeDriver == ControllerType.JOYSTICK
						? driverJoystick.getRawButton(ControllerIO.getTrigger())
						: true,
				() -> driverJoystick.getRawButton(XboxController.Button.kLeftBumper.value),
				() -> driverJoystick.getRawButton(XboxController.Button.kRightBumper.value)));

		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate
	 */
	public void configureBindings() {
		new JoystickButton(driverJoystick, XboxController.Button.kA.value).whileTrue(new Pickup(intake));
		new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileTrue(new Climb(climber));
		new JoystickButton(driverJoystick, XboxController.Button.kX.value).whileTrue(new Shoot(shooter));
	}

	public Command getAutonomousCommand() {
		return null; // Placeholder until we develop automonous commands
	}

	public static Joystick getDriverJoystick() {
		return driverJoystick;
	}

	public static Joystick getOperatorJoystick() {
		return operatorJoystick;
	}
}
