// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.RobotContainer.*;

public class NavXGyro extends SubsystemBase {

    private static NavXGyro instance;

    public static AHRS navX;
    public static double zeroHeading;
    public static double zeroAngle;

    /** Creates a new NavXGyro. */
    public NavXGyro() {
        navX = new AHRS(SPI.Port.kMXP);

        zeroHeading = getNavHeading();
        zeroAngle = getNavAngle();
        System.out.println("Setup ZeroAngle " + zeroAngle);

    }

    // Public Methods
    public static NavXGyro getInstance() {
        if (instance == null) {
            instance = new NavXGyro();
        }
        return instance;
    }

    public double getNavHeading() {
        double heading = navX.getFusedHeading();
        return heading;
    }
    
    public double getNavPitch() {
        double pitch = navX.getPitch();
        return pitch;
    }

    public double getNavAngle() {
        double angle = navX.getAngle();
        return angle;
    }

    public void zeroNavHeading() {
        //navX.zeroYaw();
        navX.reset();
        zeroHeading = getNavHeading();
        zeroAngle = getNavAngle();
        System.out.println("ZeroHeading: " + zeroHeading);
        System.out.println("ZeroAngle: " + zeroAngle);
    }

    public double getZeroHeading(){
        return zeroHeading;
    }

    public double getZeroAngle(){
        return zeroAngle;
    }

    public Rotation2d getNavXRotation2D(){
        return Rotation2d.fromDegrees(navX.getAngle());
    }

    public double getHeading() {
        return Math.IEEEremainder(-getNavAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetGyro(){
        navX.reset();
    }



    double navXGs;

    /*
    @Override
    public void periodic(){

        navXGs = Math.sqrt( (navX.getRawAccelX() * navX.getRawAccelX()) + (navX.getRawAccelY() * navX.getRawAccelY()) + (navX.getRawAccelZ() - 9.8 * navX.getRawAccelZ() - 9.8) );

        SmartDashboard.putNumber("accel gs", navXGs);

        if(navXGs > 1){

            driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);

        }else {
            driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }

    } */

}
