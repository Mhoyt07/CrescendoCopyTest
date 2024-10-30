// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionSubsystem;

public class PodiumAlignment extends Command {
  private final SwerveDrive driveSwerve;
  private boolean isFinished;
  private double xcoord;
  private final Joystick driverL;
  private final Joystick driverR;
  double error;
  VisionSubsystem camera;
  /** Creates a new PodiumAlignment. */
  public PodiumAlignment(SwerveDrive driveSwerve, Joystick driverL, Joystick driverR, double xcoord, VisionSubsystem camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSwerve = driveSwerve;
    addRequirements(this.driveSwerve);
    isFinished = false;
    this.driverL = driverL;
    this.driverR = driverR;
    this.xcoord = xcoord;
    error = xcoord;
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //error = this.camera.getError();
    new Drive(driveSwerve, driverL, driverR, 0, 0, this.driveSwerve.alignPID.calculate(error*Constants.cameraFOVRatio, 0)*.2, true);

    if (Math.abs(error) <= 5) {
      new Drive(driveSwerve, this.driverL, this.driverR, 0, 0, 0, true);
      this.isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
