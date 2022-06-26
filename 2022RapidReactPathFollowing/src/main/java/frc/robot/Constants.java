// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static double ACUTAL_WHEEL_DIAMETER = Units.inchesToMeters(4.1966);
	public final static double ENCODER_CONVERSION_FACTOR = (Math.PI * ACUTAL_WHEEL_DIAMETER) / 1024;
    public static double ksVolts = 0.15071;
    public static double kvVoltSecondsPerMeter = 1.5545;
    public static double kaVoltSecondsSquaredPerMeter = 0.32436;
	public static double kTrackWidthMeters = 0.69; //distance between wheels horizontal
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    public static double kMaxSpeedMetersPerSecond = 3;
    public static double kMaxAccelerationMetersPerSecondSquared = 3;
    public static double kRamseteB = 2;
    public static double kRamseteZeta = 0.7;
    public static double kPDriveVel = 2.1468;
	public static double kMetersPerRotation = ACUTAL_WHEEL_DIAMETER * Math.PI; // 0.3348
    public static double kEncoderDistancePerPulse = kMetersPerRotation/4; 
}
//Meters per rotation 0.3191858136

