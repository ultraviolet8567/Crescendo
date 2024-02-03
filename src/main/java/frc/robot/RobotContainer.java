package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.KMeans;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Test;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. The structure
 * of the robot (including subsystems, commands, and trigger mappings) should be
 * declared here.
 */
public class RobotContainer {
	// Subsystems
	private static final Arm arm = new Arm();
	private static final Climber climber = new Climber();
	private static final Intake intake = new Intake();
	private static final Odometry odometry = new Odometry();
	private static final Shooter shooter = new Shooter();
	// private static final Swerve swerve = new Swerve();
	private static final Vision vision = new Vision();
	private static final KMeans kmeans = new KMeans();
	private static final Test test = new Test(kmeans);

	// Shuffleboard, haha
	GenericEntry alliance = Shuffleboard.getTab("Horrible, horrible Shuffleboard").add("Alliance", false)
			.withWidget("Toggle Button").getEntry();

	// Joysticks
	private static final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
	private static final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

	public RobotContainer() {
		// Configure default commands for driving and arm movement
		/*
		 * swerve.setDefaultCommand(new SwerveTeleOp(swerve, odometry, () ->
		 * ControllerIO.inversionY() *
		 * driverJoystick.getRawAxis(ControllerIO.getLeftY()), () ->
		 * ControllerIO.inversionX() *
		 * driverJoystick.getRawAxis(ControllerIO.getLeftX()), () ->
		 * ControllerIO.inversionRot() *
		 * driverJoystick.getRawAxis(ControllerIO.getRot()), () ->
		 * OIConstants.controllerTypeDriver == ControllerType.JOYSTICK ?
		 * driverJoystick.getRawButton(ControllerIO.getTrigger()) : true, () ->
		 * driverJoystick.getRawButton(XboxController.Button.kLeftBumper.value), () ->
		 * driverJoystick.getRawButton(XboxController.Button.kRightBumper.value)));
		 */
		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate
	 */
	private void configureBindings() {
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
