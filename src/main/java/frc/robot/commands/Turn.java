// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class Turn extends CommandBase {
  /** Creates a new Turn. */

  private DriveTrain driveTrain;
  private double angle;
  private double speed;

  public Turn(DriveTrain dt, double angle, double speed) {
    this.angle = angle;
    this.speed = speed;
    driveTrain = dt;
    addRequirements(driveTrain);


    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (angle > 0){
      driveTrain.tankDrive(speed, speed*-1);
    }
    else {
      driveTrain.tankDrive(speed*-1, speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return angle < driveTrain.getAngle();
  }

}// end of class