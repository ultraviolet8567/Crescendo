package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.ControllerIO;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. The structure
 * of the robot (including subsystems, commands, and trigger mappings) should be
 * declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Arm arm;
	// private final Climber climber;
	private final Intake intake;
	private final Odometry odometry;
	private final Shooter shooter;
	private final Swerve swerve;
	private final Gyrometer gyro;

	// Joysticks
	private static final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
	private static final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

	// THIS CODE IS TEMPORARY AND NOT FINALIZED - DELETE ON A WHIM IF IT CAUSES ANY
	// PROBLEMS
	private static final CommandXboxController operatorController = new CommandXboxController(
			OIConstants.kOperatorControllerPort);

	// Camera
	public final UsbCamera camera = CameraServer.startAutomaticCapture(0);

	// Auto chooser
	public final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		// Create the subsystems with real or simulated hardware depending on current
		// mode
		switch (Constants.currentMode) {
			case REAL -> {
				arm = new Arm(new ArmIOSparkMax());
				// climber = new Climber(new ClimberIOSparkMax());
				intake = new Intake(new IntakeIOSparkMax());
				odometry = new Odometry();
				shooter = new Shooter(new ShooterIOSparkMax(), arm);
				swerve = new Swerve();
				gyro = new Gyrometer(swerve);
			}
			case SIM -> {
				arm = new Arm(new ArmIOSim());
				// climber = new Climber(new ClimberIOSim());
				intake = new Intake(new IntakeIOSim());
				odometry = new Odometry();
				shooter = new Shooter(new ShooterIOSim(), arm);
				swerve = new Swerve();
				gyro = new Gyrometer(swerve);
			}
			default -> {
				arm = new Arm(new ArmIO() {
				});
				// climber = new Climber(new ClimberIO() {});
				intake = new Intake(new IntakeIO() {
				});
				odometry = new Odometry();
				shooter = new Shooter(new ShooterIO() {
				}, arm);
				swerve = new Swerve();
				gyro = new Gyrometer(swerve);
			}
		}

		// Configure default commands for driving and arm movement
		swerve.setDefaultCommand(new SwerveTeleOp(swerve, gyro,
				() -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getLeftY()),
				() -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getLeftX()),
				() -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
				() -> OIConstants.controllerTypeDriver == ControllerType.JOYSTICK
						? driverJoystick.getRawButton(ControllerIO.getTrigger())
						: true,
				() -> driverJoystick.getRawButton(XboxController.Button.kLeftBumper.value),
				() -> driverJoystick.getRawButton(XboxController.Button.kRightBumper.value)));

		arm.setDefaultCommand(new MoveArm(arm, () -> operatorJoystick.getRawAxis(ControllerIO.getLeftY())));

		configureBindings();

		// Post webcam feed to Shuffleboard
		Shuffleboard.getTab("Main").add("Camera", camera).withWidget(BuiltInWidgets.kCameraStream).withSize(4, 4)
				.withProperties(Map.of("rotation", "HALF"));

		// Configure the PathPlanner auto-builder
		AutoBuilder.configureHolonomic(gyro::getPose, gyro::resetPose, swerve::getRobotRelativeSpeeds,
				swerve::setModuleStates, DriveConstants.kHolonomicConfig, () -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == Alliance.Red;
					}
					return false;
				}, swerve);

		// Post auto selector to Shuffleboard
		autoChooser = AutoBuilder.buildAutoChooser("DoNothing");
		Shuffleboard.getTab("Main").add("Auto", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1);

	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate
	 */

	public void configureBindings() {
		new JoystickButton(driverJoystick, XboxController.Button.kStart.value)
				.onTrue(new InstantCommand(() -> gyro.reset()));

		new JoystickButton(operatorJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new Pickup(intake));
		new JoystickButton(operatorJoystick, XboxController.Button.kBack.value).whileTrue(new Drop(intake));
		new JoystickButton(operatorJoystick, XboxController.Button.kRightBumper.value)
				.whileTrue(new Shoot(shooter, intake));

		new JoystickButton(operatorJoystick, XboxController.Button.kB.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.TAXI)));
		new JoystickButton(operatorJoystick, XboxController.Button.kA.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.ROOMBA)));
		new JoystickButton(operatorJoystick, XboxController.Button.kY.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SPEAKER)));
		new JoystickButton(operatorJoystick, XboxController.Button.kX.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.AMP)));
		// new JoystickButton(operatorJoystick, XboxController.Button.kStart.value)
		// .onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.TRAP)));

		// new JoystickButton(driverJoystick, XboxController.Button.kX.value)
		// .onTrue(new InstantCommand(() -> Lights.getInstance().hasNote =
		// !Lights.getInstance().hasNote));

		// new JoystickButton(operatorJoystick, XboxController.Button.kStart.value)
		// .whileTrue(new Climb(climber, "extend"));
		// new JoystickButton(operatorJoystick, XboxController.Button.kBack.value)
		// .whileTrue(new Climb(climber, "retract"));

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
		return autoChooser.getSelected();
	}

	public static Joystick getDriverJoystick() {
		return driverJoystick;
	}

	public static Joystick getOperatorJoystick() {
		return operatorJoystick;
	}
}
