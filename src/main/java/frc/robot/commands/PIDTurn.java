// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;

public class PIDTurn extends CommandBase {
  /** Creates a new Turn. */

  private DriveTrain driveTrain;
  private double angle;
  private double speed;
  private int constant = 1;
  private double error;
  private double kP=0.5;
  private double kI=0.5;
  private double kD=0.5;
  private double P;
  private double I;
  private double D;
  private Timer Timer;
  private double lastError;
  private double errorSum;
  private double lastTimeStamp = 0;
  


  public PIDTurn(DriveTrain dt, double angle) {
    Timer = new Timer();

    this.angle = angle;
    if (angle >= 0){
      constant = 1;
    }
    else{
      constant = -1;
    }
  
    driveTrain = dt;
    addRequirements(driveTrain);


    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.reset();
    Timer.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = angle - driveTrain.getAngle();
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    errorSum += error *dt;
    double outputAngle = kP * error + kI * errorSum;
 
    P = (error/angle);
    I = errorSum;
    D = (error - lastError)/dt;
    speed = 0.8 * (kP * P + kI * I + kD * D);

    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = error;

    /*
    error = error / angle;
    speed = error * 0.7;
    */
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return angle < driveTrain.getAngle();
  }

}// end of class