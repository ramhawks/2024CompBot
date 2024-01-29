// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
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

  public final SparkPIDController positionMotorPID;
  public final SparkPIDController outerMotorPID;
  public final SparkPIDController innerMotorPID;

  public final ColorSensorV3 proxSensor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private double proxDistance;


  /** Creates a new feederSubsystem. */
  public feederSubsystem() {

    ///****************outer motor stuff***************/
    outerMotor = new CANSparkMax(Feeder.outerMotorID, MotorType.kBrushless);
    outerMotor.restoreFactoryDefaults();
    outerMotor.setSmartCurrentLimit(Feeder.outerMotorCurrentLimit); 
    
    outerMotorEncoder = outerMotor.getEncoder();
    outerMotorPID = outerMotor.getPIDController();
    outerMotorPID.setP(Feeder.outerMotorKP);
    outerMotorPID.setI(Feeder.outerMotorKI);
    outerMotorPID.setD(Feeder.outerMotorKD);
    outerMotorPID.setOutputRange(Feeder.outerMotorMIN,Feeder.outerMotorMAX);
    ///****************end outer motor stuff***************/

    ///****************inner motor stuff***************/
    innerMotor = new CANSparkMax(Feeder.innerMotorID, MotorType.kBrushless);
    innerMotor.restoreFactoryDefaults();
    innerMotor.setSmartCurrentLimit(Feeder.innerMotorCurrentLimit);

    innerMotorEncoder = innerMotor.getEncoder();
    innerMotorPID = innerMotor.getPIDController();
    innerMotorPID.setP(Feeder.innerMotorKP);
    innerMotorPID.setI(Feeder.innerMotorKI);
    innerMotorPID.setD(Feeder.innerMotorKD);
    innerMotorPID.setOutputRange(Feeder.innerMotorMIN,Feeder.innerMotorMAX);
    ///****************end inner motor stuff***************/


    ///****************position motor stuff***************/
    positionMotor = new CANSparkMax(Feeder.positionMotorID, MotorType.kBrushless);
    positionMotor.restoreFactoryDefaults();
    positionMotor.setSmartCurrentLimit(Feeder.positionMotorCurrentLimit);

    positionEncoder = positionMotor.getEncoder();
    positionMotorPID= positionMotor.getPIDController();
    positionMotorPID.setP(Feeder.positionMotorKP);
    positionMotorPID.setI(Feeder.positionMotorKI);
    positionMotorPID.setD(Feeder.positionMotorKD);
    ///****************end position motor stuff***************/


    
    
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
    //gets the feeder into position for the intake to take in the "notes"

  }

  public void ampPosition(){
    //gets the feeder into position for the intake to deposit the "notes" in the amp
  }

  public void travelPosition(){
    //sets the feeder into a low enough position to travel underneath the stage

  }

  public void intake(){
    //intake takes in the "notes"
    intakePosition();

    outerMotorPID.setReference(Feeder.intakeSpeed,  CANSparkMax.ControlType.kVelocity);
    innerMotorPID.setReference(Feeder.intakeSpeed, CANSparkMax.ControlType.kVelocity);

    while (proxDistance < Feeder.intakeStopDistance) {
      
    }

    outerMotorPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    innerMotorPID.setReference(0, CANSparkMax.ControlType.kVelocity);

    outerMotor.setIdleMode(IdleMode.kBrake);
    innerMotor.setIdleMode(IdleMode.kBrake);

    return;

  }

  public void depositAmp(){
    //deposits the "notes" into the amp 
    ampPosition();

    outerMotorPID.setReference(Feeder.depositSpeed,  CANSparkMax.ControlType.kVelocity);
    innerMotorPID.setReference(Feeder.depositSpeed, CANSparkMax.ControlType.kVelocity);




  }

  public void shootSpeaker(){
    //shoots speaker

    //NON-PRIORITY, focus on getting amp depositing working well first

  }
}
