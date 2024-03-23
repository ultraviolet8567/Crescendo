// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Odometry;
// import frc.robot.subsystems.Swerve;

// public class YawAlign extends Command {
// private final Swerve swerve;
// private final Odometry odometry;
// private final PIDController controller;

// public YawAlign(Swerve swerve, Odometry odometry) {
// this.swerve = swerve;
// this.odometry = odometry;

// controller = new PIDController(0.5, 0, 0);
// }

// @Override
// public void initialize() {
// controller.reset();

// controller.setSetpoint(odometry.getSpeakerHeading().getRadians());
// }

// @Override
// public void execute() {
// double omega = controller.calculate(odometry.getHeading().getRadians());
// swerve.setModuleStates(new ChassisSpeeds(0.0, 0.0, omega));
// }

// @Override
// public void end(boolean interrupted) {
// swerve.stopModules();
// }
// }
