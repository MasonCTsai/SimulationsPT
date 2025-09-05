// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sims.mechanisms.Mechanator;
import frc.robot.sims.simulations.ElevatorSimulation;

public class ExampleSubsystem extends SubsystemBase implements Reportable {
  private final TalonFX motor;

  private double desiredPosition = 0.0;
  private boolean enabled = true;
  private MotionMagicVoltage motionMagicVoltage;
  private final NeutralOut neutralRequest = new NeutralOut();

  public ExampleSubsystem(){
    motor = new TalonFX(0);
    motionMagicVoltage = new MotionMagicVoltage(0);

    setMotorConfigs();

    CommandScheduler.getInstance().registerSubsystem(this);
    //TODO create ligament
    //TODO create simulation object
  }

  public void setMotorConfigs(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        //TODO motor configs
        // config.Feedback. // ratios
        // config.Slot0. // kP, kI, kD
        // config.MotionMagic. // kP, kI, kD
        
        StatusCode statusCode = motor.getConfigurator().apply(config);
        if (!statusCode.isOK()){
            DriverStation.reportError("Could not apply motor configs, " + statusCode.getDescription(), true);
        }
  }

  @Override
  public void periodic() {
    if (!enabled) {
      return;
    }

    motor.setControl(motionMagicVoltage.withPosition(desiredPosition));
  }


  public void setEnabled(boolean enabled){
    this.enabled = enabled;
    if(!enabled){
      motor.setControl(neutralRequest);
    } 
  }

  public void setTargetPosition(double position){
    desiredPosition = position;
  }

  public double getPosition(){
    return motor.getPosition().getValueAsDouble();
  }

  public double getTargetPosition(){
    return desiredPosition;
  }

  public boolean atSpeed(){
    return getPosition() > getTargetPosition();
  }

  @Override
  public void initShuffleboard(LOG_LEVEL priority) {}
}
