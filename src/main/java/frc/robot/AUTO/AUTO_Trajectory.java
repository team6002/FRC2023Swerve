// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SUB_Drivetrain;

public class AUTO_Trajectory {
    
    private SUB_Drivetrain m_drivetrain;
    public Trajectory testTrajectory;

    public AUTO_Trajectory(SUB_Drivetrain drivetrain){
        m_drivetrain = drivetrain;

        TrajectoryConfig config =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);   
      
        //Rotation2d uses RADIANS NOT DEGREES!
        //Use Rotation2d.fromDegrees(desiredDegree) instead
        testTrajectory =
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(Units.inchesToMeters(30), 0, new Rotation2d(0)),
        config);
      
    }
  
    public Command driveTrajectory(Trajectory trajectory) {
     
        // Create config for trajectory
        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(1);

        SwerveControllerCommand SUB_ControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            m_drivetrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drivetrain::setModuleStates,
            m_drivetrain);

        // Reset odometry to the starting pose of the trajectory.
        m_drivetrain.resetOdometry(trajectory.getInitialPose());
        
        // Run path following command, then stop at the end.
        return SUB_ControllerCommand.andThen(() -> m_drivetrain.drive(0.0 ,0.0 ,0.0, true));
    }

}