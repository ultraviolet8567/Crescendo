package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class AutoDriveOut extends Command {
    private final Swerve swerve;
    private final Timer timer;

    private final double forwardSpeed = 0.2;
    private final double movementTime = 8.0;

    public AutoDriveOut(Swerve swerve) {
        this.swerve = swerve;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println("[Auto] Running drive out auto at 20% speed");

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardSpeed, 0, 0);
        
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(moduleStates);

        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() >= movementTime) {
            swerve.stopModules();
        }
    }
}
