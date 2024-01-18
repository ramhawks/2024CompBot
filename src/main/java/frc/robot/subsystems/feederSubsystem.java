// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Feeder;

public class feederSubsystem extends SubsystemBase {

  public final CANSparkMax outerMotor;
  public final CANSparkMax innerMotor;
  public final CANSparkMax positionMotor;

  public final RelativeEncoder outerMotorEncoder;
  public final RelativeEncoder innerMotorEncoder;
  public final RelativeEncoder positionEncoder;

  public final SparkPIDController positionPID;
  public final SparkPIDController outerMotorPID;
  public final SparkPIDController innerMotorPID;

  public final ColorSensorV3 proxSensor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private double proxDistance;


  /** Creates a new feederSubsystem. */
  public feederSubsystem() {

    outerMotor = new CANSparkMax(Feeder.outerMotorID, MotorType.kBrushless);
    outerMotor.restoreFactoryDefaults();
    outerMotorEncoder = outerMotor.getEncoder();
    outerMotorPID = outerMotor.getPIDController();
    outerMotorPID.setP(Feeder.outerMotorKP);
    outerMotorPID.setI(Feeder.outerMotorKI);
    outerMotorPID.setD(Feeder.outerMotorKD);
    outerMotorPID.setOutputRange(Feeder.outerMotorMIN,Feeder.outerMotorMAX);


    innerMotor = new CANSparkMax(Feeder.innerMotorID, MotorType.kBrushless);
    innerMotor.restoreFactoryDefaults();
    innerMotorEncoder = innerMotor.getEncoder();
    innerMotorPID = innerMotor.getPIDController();


    positionMotor = new CANSparkMax(Feeder.positionMotorID, MotorType.kBrushless);
    positionMotor.restoreFactoryDefaults();
    positionEncoder = positionMotor.getEncoder();
    positionPID = positionMotor.getPIDController();

    
    
    proxSensor = new ColorSensorV3(i2cPort);


    outerMotor.burnFlash();
    innerMotor.burnFlash();
    positionMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    proxDistance = proxSensor.getProximity();
  }

  public void homeFeeder(){
    
  }

  public void intakePosition(){

  }

  public void ampPosition(){

  }

  public void travelPosition(){

  }

  public void intake(){

  }

  public void depositAmp(){

  }

  public void shootSpeaker(){

  }
}
