package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Drop;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.MoveArm;
import frc.robot.commands.Pickup;
import frc.robot.commands.RetractClimber;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.commands.autos.AutoIntake;
import frc.robot.commands.autos.AutoIntakeTimed;
import frc.robot.commands.autos.AutoRetract;
import frc.robot.commands.autos.AutoSetArmMode;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.subsystems.AutoChooser;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
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
	private final AutoChooser autoChooser;

	// Joysticks
	private static final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
	private static final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);
	private static final CommandXboxController driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private static final CommandXboxController operatorController = new CommandXboxController(
			OIConstants.kOperatorControllerPort);

	// Camera
	public final UsbCamera driverCam = CameraServer.startAutomaticCapture(0);

	public RobotContainer() {
		// Driver cam limitations
		if (RobotBase.isReal()) {
			driverCam.setFPS(60);
			driverCam.setResolution(320, 240);
		}

		// Create subsystems with real or simulated hardware depending on current mode
		switch (Constants.currentMode) {
			case REAL -> {
				arm = new Arm(new ArmIOSparkMax());
				climber = new Climber(new ClimberIOSparkMax());
				intake = new Intake(new IntakeIOSparkMax());
				shooter = new Shooter(new ShooterIOSparkMax(), arm);
			}
			case SIM -> {
				arm = new Arm(new ArmIOSim());
				climber = new Climber(new ClimberIOSim());
				intake = new Intake(new IntakeIOSim());
				shooter = new Shooter(new ShooterIOSim(), arm);
			}
			default -> {
				arm = new Arm(new ArmIO() {
				});
				climber = new Climber(new ClimberIO() {
				});
				intake = new Intake(new IntakeIO() {
				});
				shooter = new Shooter(new ShooterIO() {
				}, arm);
			}
		}
		// kmeans = new KMeans();
		swerve = new Swerve();
		// gyro = new Gyrometer(swerve);
		// vision = new Vision();
		odometry = new Odometry(swerve);

		// Configure the PathPlanner auto-builder
		AutoBuilder.configureHolonomic(odometry::getGyrometerPose, odometry::resetGyrometerPose,
				swerve::getRobotRelativeSpeeds, swerve::setModuleStates, DriveConstants.kHolonomicConfig, () -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == Alliance.Red;
					}
					return false;
				}, swerve);

		// Register PathPlanner named commands
		NamedCommands.registerCommand("AutoShoot", new AutoShoot(shooter, intake));
		NamedCommands.registerCommand("RampUp", new InstantCommand(() -> shooter.shoot()));
		NamedCommands.registerCommand("FirstShot", new InstantCommand(() -> shooter.shoot(0.67)));
		NamedCommands.registerCommand("Pickup", new AutoIntake(intake));
		NamedCommands.registerCommand("PickupTimed", new AutoIntakeTimed(intake));
		NamedCommands.registerCommand("AutoRetract", new AutoRetract(intake));
		NamedCommands.registerCommand("TaxiPosition", new AutoSetArmMode(arm, ArmMode.TAXI, 0.05));
		NamedCommands.registerCommand("AmpPosition", new AutoSetArmMode(arm, ArmMode.AMP, 0.05));
		NamedCommands.registerCommand("IntakePosition", new AutoSetArmMode(arm, ArmMode.ROOMBA, 0.01));
		NamedCommands.registerCommand("SpeakerFrontPosition", new AutoSetArmMode(arm, ArmMode.SPEAKERFRONT, 0.1));
		NamedCommands.registerCommand("SpeakerAnglePosition", new AutoSetArmMode(arm, ArmMode.SPEAKERANGLE, 0.1));
		NamedCommands.registerCommand("SpeakerStagePosition", new AutoSetArmMode(arm, ArmMode.SPEAKERSTAGE, 0.1));

		// Create AutoChooser
		autoChooser = new AutoChooser();

		// Configure default commands for driving and arm movement
		swerve.setDefaultCommand(new SwerveTeleOp(swerve, odometry,
				() -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getLeftY()),
				() -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getLeftX()),
				() -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
				() -> OIConstants.controllerTypeDriver == ControllerType.JOYSTICK
						? driverJoystick.getRawButton(ControllerIO.getTrigger())
						: true,
				() -> driverJoystick.getRawButton(XboxController.Button.kRightBumper.value)));

		arm.setDefaultCommand(new MoveArm(arm, () -> operatorJoystick.getRawAxis(ControllerIO.getLeftY())));

		// Configure button bindings
		configureBindings();

		// Post webcam feed to Shuffleboard
		Shuffleboard.getTab("Main").add("Camera", driverCam).withWidget(BuiltInWidgets.kCameraStream).withSize(4, 4)
				.withPosition(5, 0);
	}

	public void configureBindings() {
		// Button bindings
		driverController.leftBumper().whileTrue(new InstantCommand(() -> shooter.shoot(0.75)))
				.onFalse(new InstantCommand(() -> shooter.stop()));

		// driverController.y().whileTrue(new WheelRadiusCharacterization(swerve,
		// odometry,
		// WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE));
		// driverController.a().whileTrue(
		// new WheelRadiusCharacterization(swerve, odometry,
		// WheelRadiusCharacterization.Direction.CLOCKWISE));

		operatorController.leftBumper().whileTrue(new Pickup(intake));
		operatorController.leftTrigger(0.5).whileTrue(new Drop(intake));
		operatorController.rightTrigger().whileTrue(new Shoot(shooter, intake));

		operatorController.back().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SOURCE)));
		operatorController.start().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SPEAKERSTAGE)));
		operatorController.a().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.ROOMBA)));
		operatorController.b().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.AMP)));
		operatorController.x().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.TAXI)));
		operatorController.y().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SPEAKERFRONT)));

		operatorController.rightStick().onTrue(new InstantCommand(() -> intake.toggleSensorDisabled()));

		operatorController.povUp().whileTrue(new ExtendClimber(climber, "Left"))
				.onFalse(new InstantCommand(() -> climber.stopLeft()));
		operatorController.povRight().whileTrue(new ExtendClimber(climber, "Right"))
				.onFalse(new InstantCommand(() -> climber.stopRight()));
		operatorController.povLeft().whileTrue(new RetractClimber(climber, "Right"))
				.onFalse(new InstantCommand(() -> climber.stopRight()));
		operatorController.povDown().whileTrue(new RetractClimber(climber, "Left"))
				.onFalse(new InstantCommand(() -> climber.stopLeft()));

		// Overrides
		// driverController.back().onTrue(new InstantCommand(() ->
		// odometry.resetPose(Constants.speaker)));
		driverController.start().onTrue(new InstantCommand(() -> odometry.resetGyrometerHeading()));
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelectedAuto();
	}

	public void setInitialGyroYaw() {
		odometry.setGyroYaw(autoChooser.getAutoStartingPose().getRotation());
	}

	public static XboxController getDriverJoystick() {
		return driverController.getHID();
	}

	public static XboxController getOperatorJoystick() {
		return operatorController.getHID();
	}
}
