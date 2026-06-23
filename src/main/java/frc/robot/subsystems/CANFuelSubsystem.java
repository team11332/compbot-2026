// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

import java.security.Policy;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX feederRoller;
  private final TalonFX intakeLauncherRoller;
  private final SparkMax Indexer;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new TalonFX(INTAKE_LAUNCHER_MOTOR_ID);
    feederRoller = new TalonFX(FEEDER_MOTOR_ID);
    Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    Indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VELOCITY);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VELOCITY);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VELOCITY);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VELOCITY);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VELOCITY);
   
    // create theget configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();
    launcherConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    launcherConfig.CurrentLimits.SupplyCurrentLimit = LAUNCHER_MOTOR_CURRENT_LIMIT;
    intakeLauncherRoller.getConfigurator().apply(launcherConfig);
  }

   
// A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    Indexer.set(power); // positive for shooting
  }
  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller.setFeederRoller( (-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VELOCITY)));
    intakeLauncherRoller.setControl(new VelocityVoltage(-1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VELOCITY)).withSlot(0));
;
    intakeLauncherRoller
        .setControl(new VelocityVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VELOCITY)).withSlot(0));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller
        .setControl(new VelocityVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VELOCITY)).withSlot(0));
    intakeLauncherRoller
        .setControl(new VelocityVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VELOCITY)).withSlot(0));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederRoller
        .setControl(new VelocityVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VELOCITY)).withSlot(0));
    intakeLauncherRoller
        .setControl(new VelocityVoltage(SmartDashboard.getNumber("Spin-up launcher roller value", LAUNCHING_LAUNCHER_VELOCITY)).withSlot(0));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
