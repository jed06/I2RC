// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path1 extends SequentialCommandGroup {
  /** Creates a new DriveForwardSequence. */
  // PATH FOR MAKING SQUARE
  public Path1(DriveTrain driveTrain){
    addCommands(new DriveForward(driveTrain, 5, 0.3));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //super(new DriveForward(driveTrain,30, 0.3));
    
  }

 
}

