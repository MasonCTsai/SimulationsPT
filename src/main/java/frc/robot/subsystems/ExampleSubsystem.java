// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.sims.PhysicsSimulator;
import frc.robot.sims.mechanisms.Mechanator;
import frc.robot.sims.simulations.ArmSimulation;
import frc.robot.sims.simulations.ElevatorSimulation;
import static edu.wpi.first.units.Units.Volts;

public class ExampleSubsystem extends SubsystemBase implements Reportable {
  private final TalonFX motor;

  private double desiredPosition = 0.25 / 2.0;
  private boolean enabled = true;
  private MotionMagicVoltage motionMagicVoltage;
  private final NeutralOut neutralRequest = new NeutralOut();
  
  private final Joystick joy = new Joystick(0);
  // private final JoystickButton joystick = new JoystickButton(null, 0);

  public ExampleSubsystem(){
    motor = new TalonFX(0);
    motionMagicVoltage = new MotionMagicVoltage(0);

    sysIdRoutineArm = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(2.0),
        null,
        (state) -> {
          SignalLogger.writeString("state", state.toString());
          SmartDashboard.putString("stateSmartdashboard", state.toString());
        }
      ),
      new SysIdRoutine.Mechanism(
        output -> motor.setVoltage(output.baseUnitMagnitude()), 
        null, 
        this
        )
    ); 
    setMotorConfigs();

    CommandScheduler.getInstance().registerSubsystem(this);
    //TODO create ligament
    MechanismLigament2d ligament = Mechanator.getInstance().getLigament("arm", 1.5, 1.5, 1, 0.0);
    //TODO create simulation object
    ArmSimulation armSim = new ArmSimulation(motor, ligament, 100, 0.1, 0.75, Math.PI / -2.0, Math.PI * 23.0 / 2.0, 0.0);
  }

  public void setMotorConfigs(){
    TalonFXConfiguration config = new TalonFXConfiguration();
    //TODO motor configs
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = 1;

    config.Slot0.kP = 0.0; // kP, kI, kD
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 25.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kG = 0.045;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = 0.5; // kP, kI, kD
    config.MotionMagic.MotionMagicAcceleration = 1.0;
    config.MotionMagic.MotionMagicJerk = 0;
    
    StatusCode statusCode = motor.getConfigurator().apply(config);
    if (!statusCode.isOK()){
        DriverStation.reportError("Could not apply motor configs, " + statusCode.getDescription(), true);
    }
  }

  double voltage = 0.0;

  double ff = 0.0;
  @Override
  public void periodic() {
    if (!enabled) {
      return;
    }
    double v = 0.0;
    if (joy.getRawButton(1)) v = 1.0;
    if (joy.getRawButton(2)) v = -1.0;
    // if (joy.getRawButtonPressed(1)) {
    //   CommandScheduler.getInstance().schedule(sysId());
    // }
    if (joy.getRawButton(3)) voltage += 0.25 / 50.0;
    if (joy.getRawButton(4)) voltage -= 0.25 / 50.0;
    
    // if (joy.getRawButtonReleased(1)) voltage = 0.0;
    // if (joy.getRawButtonReleased(2)) voltage = 0.0;
    // if (joy.getRawButtonReleased(3)) voltage = 0.0;
    // if (joy.getRawButtonReleased(4)) voltage = 0.0;

    // motor.setVoltage(voltage);
    // ff = Math.cos(motor.getPosition().getValueAsDouble() * 2.0 * Math.PI) * voltage; //2.51689 // 0.045
    // motor.setVoltage(ff + v);
    motor.setControl(motionMagicVoltage.withPosition(desiredPosition).withFeedForward(v + ff));
  }

  final SysIdRoutine sysIdRoutineArm;

  public Command sysId() {
      return Commands.sequence(
              sysIdRoutineArm
                      .quasistatic(SysIdRoutine.Direction.kForward)
                      .until(() -> (getPosition() > 5.74)),
              sysIdRoutineArm
                      .quasistatic(SysIdRoutine.Direction.kReverse)
                      .until(() -> (getPosition() < -0.24)),
              sysIdRoutineArm
                      .dynamic(SysIdRoutine.Direction.kForward)
                      .until(() -> (getPosition() > 5.74)),
              sysIdRoutineArm
                      .dynamic(SysIdRoutine.Direction.kReverse)
                      .until(() -> (getPosition() < -0.24)));
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
  public void initShuffleboard(LOG_LEVEL priority) {
    ShuffleboardTab tab = Shuffleboard.getTab("sim");
    tab.addDouble("position", () -> motor.getPosition().getValueAsDouble());
    tab.addDouble("target position", () -> motionMagicVoltage.Position);
    tab.addDouble("velocity", () -> motor.getVelocity().getValueAsDouble());
    tab.addDouble("ff", () -> ff);
    tab.addDouble("kG", () -> voltage);
    tab.addDouble("voltage", () -> motor.getSimState().getMotorVoltage());
    tab.addDouble("battery voltage", () -> RobotController.getBatteryVoltage());
    tab.addBoolean("voltage up", () -> joy.getRawButton(1));
    tab.addBoolean("voltage down", () -> joy.getRawButton(2));
    tab.addBoolean("voltage ramp up", () -> joy.getRawButton(3));
    tab.addBoolean("voltage ramp down", () -> joy.getRawButton(4));
  }
}
