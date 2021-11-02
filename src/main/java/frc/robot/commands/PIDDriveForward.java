// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDDriveForward extends CommandBase {
  /** Creates a new DriveForward. */
  private DriveTrain driveTrain;
  private int distance;
  private double speed;
  private double error;
  public PIDDriveForward(DriveTrain dt,int distance, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;
    this.speed = speed;
    driveTrain = dt;
    addRequirements(driveTrain);
  }
  public PIDDriveForward(int distance, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;
    this.speed = speed;
    addRequirements(driveTrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
    //resetEncoders

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = distance - driveTrain.getPosition();
    error = error / distance;
    speed = error * 0.7;

    if (speed > 0.7){
      speed = 0.7;

    }

    if (speed < 0.1){
      speed = 0.1;
    }

    driveTrain.tankDrive(speed,speed);


  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
    // stop motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveTrain.getPosition() < distance);
  }

}// end of class
