// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private SwerveDriveOdometry odometry;
  private Field2d field;
  private final SwerveModule[] dt;
  private final PigeonIMU gyro = new PigeonIMU(10);
  public final PIDController alignPID = new PIDController(Constants.alignkP, 0, 0);
  public final PIDController autoPID = new PIDController(Constants.autoRotateP, 0, 0);

  private final Joystick driverL;
  private final Joystick driverR;

  // dt is DriveTrain
  public SwerveDrive(Joystick driverL, Joystick driverR) {
    // creates a "map" of the robot, recording the position of each swerve wheel
    // relative to the others

    gyro.configFactoryDefault();
    zeroGyro();
    


    this.dt = new SwerveModule[] {
        new SwerveModule(0, Constants.mod0DriveMotor, Constants.mod0TurningMotor, Constants.mod0CANCoder,
            Constants.mod0TurningOffset),
        new SwerveModule(1, Constants.mod1DriveMotor, Constants.mod1TurningMotor, Constants.mod1CANCoder,
            Constants.mod1TurningOffset),
        new SwerveModule(2, Constants.mod2DriveMotor, Constants.mod2TurningMotor, Constants.mod2CANCoder,
            Constants.mod2TurningOffset),
        new SwerveModule(3, Constants.mod3DriveMotor, Constants.mod3TurningMotor, Constants.mod3CANCoder,
            Constants.mod3TurningOffset)
    };
    odometry = new SwerveDriveOdometry(Constants.SwerveMap, getYaw(), new SwerveModulePosition[] {dt[0].getPosition(),
       dt[1].getPosition(), dt[2].getPosition(), dt[3].getPosition()
    }); 

    this.driverL = driverL;
    this.driverR = driverR;

    Timer.delay(1);
    resetToAbsolute2();

    field = new Field2d();

  }

  public void drive(Translation2d translation, double rotation, boolean isFieldRelative, boolean isAuto) {
    //translation.getAngle()
    SmartDashboard.putNumber("Translation Angle", translation.getAngle().getDegrees());

    SwerveModuleState[] swerveModuleStates = Constants.SwerveMap
        .toSwerveModuleStates(isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(), translation.getY(), rotation, getYaw())
          : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

    for (SwerveModule module : this.dt) {
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isAuto);
    }

  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxAutoSpeed);

    for (SwerveModule module : this.dt) {
      module.setDesiredState(desiredStates[module.moduleNumber], true);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : dt) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void resetToAbsolute2() {
    for (SwerveModule module : dt) {
      module.resetToAbsolute();
    }
  }

  public void testDrive() {
    for (SwerveModule module : dt) {
      module.gogogo();
    }
  }

  public void resetOdometry(Pose2d odoPose) {
    odometry.resetPosition(getYaw(), new SwerveModulePosition[] {dt[0].getPosition(),
      dt[1].getPosition(), dt[2].getPosition(), dt[3].getPosition()
   },  odoPose);
  }

  

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public void zeroGyroAuto(double zeroPos) {
    gyro.setYaw(zeroPos);
  }

  public Rotation2d getYaw() {

    return (false)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    odometry.update(getYaw(), new SwerveModulePosition[] {dt[0].getPosition(),
      dt[1].getPosition(), dt[2].getPosition(), dt[3].getPosition()
   });
    field.setRobotPose(getPose());
    for (SwerveModule module : dt) {
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Cancoder", module.getCANCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Integrated", module.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
      // SmartDashboard.putNumber("Joystick1 X", this.driverL.getRawAxis(0));
      // SmartDashboard.putNumber("Joystick1 Y", this.driverL.getRawAxis(1));
      // SmartDashboard.putNumber("Joystick2 X", this.driverR.getRawAxis(0));
      // SmartDashboard.putNumber("Joystick2 Y", this.driverR.getRawAxis(1));
      SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees()%360);
      SmartDashboard.putNumber("Joystick Hat", this.driverL.getPOV());
      
      



      //SmartDashboard.putNumber("Mod" + module.moduleNumber + "turningPosition", module.turningEncoder.getPosition());
      //SmartDashboard.putNumber("Mod 0 CANCoder Absolute", module.bestTurningEncoder.getAbsolutePosition()+ 148.2);



    }
  }
}
