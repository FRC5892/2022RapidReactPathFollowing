// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
	private CANSparkMax leftMotor1 = driveMotor(1, true);
	private CANSparkMax leftMotor2 = driveMotor(2, true);
	private CANSparkMax leftMotor3 = driveMotor(3, true);
	private CANSparkMax rightMotor1 = driveMotor(4, false);
	private CANSparkMax rightMotor2 = driveMotor(5, false);
	private CANSparkMax rightMotor3 = driveMotor(6, false);

	private Encoder m_leftEncoder = new Encoder(0, 1, true);
	private Encoder m_rightEncoder = new Encoder(2, 3, false);

	private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
	private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

	private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  private final Gyro m_gyro = new ADXRS450_Gyro();
  private final DifferentialDriveOdometry m_odometry;
  
  public Drivetrain() {
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  public CANSparkMax driveMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.setSmartCurrentLimit(40);
    return sparkMax;
  }

  public void driveWithJoysticks(double xSpeed, double zRotation) {
		drive.arcadeDrive(xSpeed, zRotation, true);
	}

  public void arcadeDrive(double xSpeed, double zRotation) {
		drive.arcadeDrive(xSpeed, zRotation, false);
	}

  @Override
  public void periodic() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    // This method will be called once per scheduler run
    m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
  }

  public void stopMotors() {
		drive.stopMotor();
	}

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetOdometry(Pose2d pose) {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
		m_odometry.resetPosition(pose, m_gyro.getRotation2d());
	  }
}
