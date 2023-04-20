// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;
import java.security.cert.TrustAnchor;

import javax.lang.model.util.ElementScanner14;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //import the drive system that we are using, arcade
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controller.LogitechController;

public class Robot extends TimedRobot 
{
  int numOfCats = 7; //cats =motor
  boolean doWeHaveCats = true;

  CANSparkMax frontLeftDriveMotor, backLeftDriveMotor, frontRightDriveMotor, backRightDriveMotor;

  CANSparkMax elevatorMotor, pivotMotor;
  
  CANSparkMax intakeMotor;

  DifferentialDrive drivetrain; //type of drivetrain

  double driver_speed = 0.61;
  double operator_speed = 0.60;

  double pullInPower = 0.8; //increase power
  double ejectPower = -0.8; //putting it down, throwing out

  boolean BButton = false;
  boolean YButton = false;
 
  LogitechController driverController = new LogitechController(0);
  LogitechController operatorController = new LogitechController(1);

  Timer autoTimer = new Timer();

  @Override
  public void robotInit() {

    //Create the drive motors
    frontLeftDriveMotor = new CANSparkMax(1, MotorType.kBrushless); //neos are brushless, sim and redlines is brushed
    backLeftDriveMotor = new CANSparkMax(9, MotorType.kBrushless);
    backLeftDriveMotor.setInverted(true);
    frontLeftDriveMotor.setInverted(true);
    frontRightDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
    backRightDriveMotor = new CANSparkMax(4, MotorType.kBrushless);

    

    // Invert Motors as needed
    frontLeftDriveMotor.setInverted(true);

    //Group the drive motors together so they can be set to the same power and not fight each other
    MotorControllerGroup leftDriveMotors = new MotorControllerGroup(backLeftDriveMotor, frontLeftDriveMotor);
    MotorControllerGroup rightDriveMotors = new MotorControllerGroup(backRightDriveMotor, frontRightDriveMotor);

    //Create a differential drive object that can easily make allow for different types of driving overall
    drivetrain = new DifferentialDrive(leftDriveMotors, rightDriveMotors);

    //Create Elevator/Pivot Motors
    elevatorMotor = new CANSparkMax(6, MotorType.kBrushed);
    pivotMotor = new CANSparkMax(5, MotorType.kBrushed);

    //Create Intake Motors
    intakeMotor = new CANSparkMax(7, MotorType.kBrushed);
  }

  @Override
  public void robotPeriodic() 
  {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {  
    autoTimer.reset();
    autoTimer.start();
  }

  public void SpillTheCube()
  {
    intakeMotor.set(ejectPower);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
   {
      if (autoTimer.get() < 1)
      {
       // SpillTheCube();
        drivetrain.arcadeDrive(-0.4, 0); 
      }

      else if (autoTimer.get() < 2)
      {
        drivetrain.arcadeDrive(0.5, 0);        
        //drive backwards
    
      }
      
      else if (autoTimer.get() < 3.25)
      {
        drivetrain.arcadeDrive(-0.65, 0);
        //drive forwards
      }
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {} //teleop=drive controller section

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    //Using arcade drive to allow squaring of inputs to have more controlled driving at slower speeds
    // drivetrain.arcadeDrive(driver_speed*driverController.getLeftY(), driver_speed*driverController.getRightX());
  
    if (!(driverController.getLeftBumper() == true ^ driverController.getRightBumper() == true))
    {
      pivotMotor.set(0.0);
    }
    if (driverController.getLeftBumper() == true)
    {
      pivotMotor.set(-1);
    }
    else if (driverController.getRightBumper() == true)
    {
      pivotMotor.set(1);
    }

    double LTrigger = operatorController.getLeftTriggerAxis();
    double RTrigger = operatorController.getRightTriggerAxis();
    double power = RTrigger;

    if (LTrigger > RTrigger)
    {
      power = -LTrigger;
    }

    if (operatorController.getBButtonPressed() == true)
    {
        BButton = !BButton;   
    }
    if(operatorController.getYButtonPressed() == true)
    {
        YButton = !YButton;
    }

    if(BButton == true ^ YButton == true)
    {
      intakeMotor.set(0.0);
    }
    else if (BButton == true)
    {
      intakeMotor.set(pullInPower);
      
    }
    else
    {
      intakeMotor.set(ejectPower);
    }

    elevatorMotor.set(operator_speed*power);
    
    intakeMotor.set(2*operator_speed*operatorController.getRightY()); //pivot=brings up and down, on the rope; elevator=extends arm

    elevatorMotor.set(operator_speed*power);

    double powerFactor = (driver_speed + (1-driver_speed) * driverController.getRightTriggerAxis() - 0.205* driverController.getLeftTriggerAxis() );
    double drivePower = powerFactor*driverController.getLeftY();
    double turnPower = 0.75*powerFactor*driverController.getRightX();
    drivetrain.arcadeDrive(drivePower, turnPower);
    
    /* System.out.println(operatorController.getRightTriggerAxis());
     double reverse = -1;
    if (operatorController.getLeftTriggerAxis() > operatorController.getRightTriggerAxis())
     {
     reverse = 1;
     }
     double speed = Math.max(operatorController.getLeftTriggerAxis(), operatorController.getRightTriggerAxis());
     intakeMotor.set(reverse*operator_speed*speed );*/

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
