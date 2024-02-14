package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.Pickup;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
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

	// THIS CODE IS TEMPORARY AND NOT FINALIZED - DELETE ON A WHIM IF IT CAUSES ANY
	// PROBLEMS
	private static final CommandXboxController operatorController = new CommandXboxController(
			OIConstants.kOperatorControllerPort);

	public RobotContainer() {
		// Create the subsystems with real or simulated hardware depending on current
		// mode
		switch (Constants.currentMode) {
			case REAL -> {
				arm = new Arm(new ArmIOSparkMax());
				climber = new Climber(new ClimberIOSparkMax());
				intake = new Intake(new IntakeIOSparkMax());
				odometry = new Odometry();
				shooter = new Shooter(new ShooterIOSparkMax());
				swerve = new Swerve();
			}
			case SIM -> {
				arm = new Arm(new ArmIOSim());
				climber = new Climber(new ClimberIOSim());
				intake = new Intake(new IntakeIOSim());
				odometry = new Odometry();
				shooter = new Shooter(new ShooterIOSim());
				swerve = new Swerve();
			}
			default -> {
				arm = new Arm(new ArmIO() {
				});
				climber = new Climber(new ClimberIO() {
				});
				intake = new Intake(new IntakeIO() {
				});
				odometry = new Odometry();
				shooter = new Shooter(new ShooterIO() {
				});
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

		arm.setDefaultCommand(new MoveArm(arm, () -> -1 * operatorJoystick.getRawAxis(ControllerIO.getRightY())));

		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate
	 */

	public void configureBindings() {
		new JoystickButton(operatorJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new Pickup(intake));
		new JoystickButton(operatorJoystick, XboxController.Button.kRightBumper.value)
				.whileTrue(new Shoot(shooter, intake));
		new JoystickButton(driverJoystick, XboxController.Button.kX.value)
				.onTrue(new InstantCommand(() -> Lights.getInstance().hasNote = !Lights.getInstance().hasNote));

		// new JoystickButton(operatorJoystick, XboxController.Button.kStart.value)
		// .whileTrue(new Climb(climber, "extend"));
		// new JoystickButton(operatorJoystick, XboxController.Button.kBack.value)
		// .whileTrue(new Climb(climber, "retract"));

		new JoystickButton(operatorJoystick, XboxController.Button.kB.value).onTrue(new SetArmAngle(arm, 1));
		new JoystickButton(operatorJoystick, XboxController.Button.kA.value).onTrue(new SetArmAngle(arm, 2));
		new JoystickButton(operatorJoystick, XboxController.Button.kY.value).onTrue(new SetArmAngle(arm, 3));
		new JoystickButton(operatorJoystick, XboxController.Button.kX.value).onTrue(new SetArmAngle(arm, 4));
		new JoystickButton(operatorJoystick, XboxController.Button.kStart.value).onTrue(new SetArmAngle(arm, 5));

		// new POVButton(operatorJoystick, -1).whileTrue(new SetArmAngle(arm, 1));
		// new POVButton(operatorJoystick, 0).whileTrue(new SetArmAngle(arm, 2));
		// new POVButton(operatorJoystick, 90).whileTrue(new SetArmAngle(arm, 3));
		// new POVButton(operatorJoystick, 180).whileTrue(new SetArmAngle(arm, 4));
		// new POVButton(operatorJoystick, 270).whileTrue(new SetArmAngle(arm, 5));

		/*
		 * NOTE: If you wish to use the TRIGGERS, try this code. NOTE: THIS IS UNTESTED
		 *
		 * operatorController.leftTrigger().onTrue(new Command());
		 * operatorController.leftTrigger().whileTrue(new Command());
		 *
		 * operatorController.a().onTrue(new Command());
		 * operatorController.rightTrigger().whileTrue(new Command());
		 *
		 * operatorController.povUp().onTrue(new Command());
		 */
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
