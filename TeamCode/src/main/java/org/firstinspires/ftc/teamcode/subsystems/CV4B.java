package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CV4B {
	/*
	 * Configurable constants
	 *
	 * Most positions/values can be changed here.
	 * 
	 * While facing same direction as robot
	 * Drive: Higher = back
	 * Coax: Higher = back
	 */
    public static double PRE_TRANSFER_DRIVE = 0.32;
	public static double PRE_TRANSFER_COAX = 0.4;
    public static double TRANSFER_DRIVE = 0.32;
    public static double TRANSFER_COAX = 0.36;
    public static double DEPOSIT_DRIVE = 0.75;
    public static double DEPOSIT_COAX = 0.5;
    public static double SPECIMEN_GRAB_DRIVE = 0.375;
    public static double SPECIMEN_GRAB_COAX = 0.4;
    public static double SPECIMEN_HANG_DRIVE = 0.75;
    public static double SPECIMEN_HANG_COAX = 0.5;
	// Offsets are added to all positions.
	// Use them to fix positions after slips
    public static double offset_drive = 0;
    public static double offset_coax = 0;

	/*
	 * DO NOT change the below code unless necessary
	 * or you know what you are doing
	 */

	private ServoImplEx cv4bLeftServo;
	private ServoImplEx cv4bRightServo;
	private ServoImplEx cv4bCoaxialServo;

    public static Positions position = Positions.TRANSFER;

    public enum Positions {
		BASE,
		TRANSFER,
		PRE_TRANSFER,
		SPECIMEN_GRAB,
		SPECIMEN_HANG,
		DEPOSIT,
		MANUAL
	} 

    public CV4B(HardwareMap hardwareMap) {
        cv4bLeftServo = hardwareMap.get(ServoImplEx.class, "cv4bLeftServo");
		cv4bRightServo = hardwareMap.get(ServoImplEx.class, "cv4bRightServo");
		cv4bLeftServo.setDirection(Servo.Direction.FORWARD);
		cv4bRightServo.setDirection(Servo.Direction.REVERSE);
		cv4bCoaxialServo = hardwareMap.get(ServoImplEx.class, "cv4bCoaxialServo");
		cv4bCoaxialServo.setDirection(Servo.Direction.REVERSE);
    }

	/*
	 * Sets the position of the CV4B
	 * 
	 * There are 2 methods: enum and manual.
	 * Enum should be the primary one used, 
	 * as it also updates a variable for telemetry
	 * and allows for quick changing of positions.
	 */

    public void setPosition(Positions position) {
        CV4B.position = position;
		double driveTarget = 0;
		double coaxTarget = 0;
		switch (position) {
			case TRANSFER:
			default:
				driveTarget = TRANSFER_DRIVE+offset_drive;
				coaxTarget = TRANSFER_COAX+offset_coax;
				break;
			case PRE_TRANSFER:
				driveTarget = PRE_TRANSFER_DRIVE+offset_drive;
				coaxTarget = PRE_TRANSFER_COAX+offset_coax;
				break;				
			case DEPOSIT:
				driveTarget = DEPOSIT_DRIVE+offset_drive;
				coaxTarget = DEPOSIT_COAX+offset_coax;
				break;
			case SPECIMEN_GRAB:
				driveTarget = SPECIMEN_GRAB_DRIVE+offset_drive;
				coaxTarget = SPECIMEN_GRAB_COAX+offset_coax;
				break;
            case SPECIMEN_HANG:
				driveTarget = SPECIMEN_HANG_DRIVE+offset_drive;
				coaxTarget = SPECIMEN_HANG_COAX+offset_coax;
				break;
		}
		cv4bLeftServo.setPosition(driveTarget);
		cv4bRightServo.setPosition(driveTarget);
		cv4bCoaxialServo.setPosition(coaxTarget);
    }

    public void setPosition(double v4bRot, double coaxialRot) {
		CV4B.position = CV4B.Positions.MANUAL;
		cv4bLeftServo.setPosition(v4bRot);
		cv4bRightServo.setPosition(v4bRot);
		cv4bCoaxialServo.setPosition(coaxialRot);
	}

	/*
	 * Sets the servo PWM
	 * 
	 * These can be used to "disengage" the servos,
	 * essentially turning off their resistance.
	 */

    public void setDrivePWM(boolean enabled) {
        if (enabled) {
            cv4bLeftServo.setPwmEnable();
            cv4bRightServo.setPwmEnable();
        } else {
            cv4bLeftServo.setPwmDisable();
            cv4bRightServo.setPwmDisable();
        }
    }

    public void setCoaxialPWM(boolean enabled) {
        if (enabled) {
            cv4bCoaxialServo.setPwmEnable();
        } else {
            cv4bCoaxialServo.setPwmDisable();
        }
    }
}