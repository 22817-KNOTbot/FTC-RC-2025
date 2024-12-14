package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CV4B {
    // While facing same direction as robot
    // Drive: Higher = back
    // Coax: Higher = back
    public static double BASE_DRIVE = 0.29;
    public static double BASE_COAX = 0.25;
    public static double SPECIMEN_GRAB_DRIVE = 0.81;
    public static double SPECIMEN_GRAB_COAX = 0.44;
    public static double SPECIMEN_HANG_DRIVE = 0.81;
    public static double SPECIMEN_HANG_COAX = 0.47;
    public static double TRANSFER_DRIVE = 0.1;
    public static double TRANSFER_COAX = 0.38;
    public static double PRE_DEPOSIT_DRIVE = 0.65;
    public static double PRE_DEPOSIT_COAX = 0.41;
    public static double DUMP_DRIVE = 0.65;
    public static double DUMP_COAX = 0.49;
    public static double offset_drive = 0.1;
    public static double offset_coax = 0.16;
	private ServoImplEx cv4bLeftServo;
	private ServoImplEx cv4bRightServo;
	private ServoImplEx cv4bCoaxialServo;
    public Positions position = Positions.BASE;

    public enum Positions {
		BASE,
		TRANSFER,
		SPECIMEN_GRAB,
		SPECIMEN_HANG,
		PRE_DEPOSIT,
		DUMP
	} 

    public CV4B(HardwareMap hardwareMap) {
        cv4bLeftServo = hardwareMap.get(ServoImplEx.class, "cv4bLeftServo");
		cv4bRightServo = hardwareMap.get(ServoImplEx.class, "cv4bRightServo");
		cv4bLeftServo.setDirection(Servo.Direction.FORWARD);
		cv4bRightServo.setDirection(Servo.Direction.REVERSE);
		cv4bCoaxialServo = hardwareMap.get(ServoImplEx.class, "cv4bCoaxialServo");
		cv4bCoaxialServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(Positions position) {
        this.position = position;
		switch (position) {
			case BASE:
				setPosition(BASE_DRIVE+offset_drive, BASE_COAX+offset_coax);
				break;
			case SPECIMEN_GRAB:
				setPosition(SPECIMEN_GRAB_DRIVE+offset_drive, SPECIMEN_GRAB_COAX+offset_coax);
				break;
            case SPECIMEN_HANG:
				setPosition(SPECIMEN_HANG_DRIVE+offset_drive, SPECIMEN_HANG_COAX+offset_coax);
				break;
			case TRANSFER:
				setPosition(TRANSFER_DRIVE+offset_drive, TRANSFER_COAX+offset_coax);
				break;
			case PRE_DEPOSIT:
				setPosition(PRE_DEPOSIT_DRIVE+offset_drive, PRE_DEPOSIT_COAX+offset_coax);
				break;
			case DUMP:
				setPosition(DUMP_DRIVE+offset_drive, DUMP_COAX+offset_coax);
				break;
		}
    }

    public void setPosition(double v4bRot, double coaxialRot) {
		cv4bLeftServo.setPosition(v4bRot);
		cv4bRightServo.setPosition(v4bRot);
		cv4bCoaxialServo.setPosition(coaxialRot);
	}

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