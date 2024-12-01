package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="CV4B testing", group="Debug")
public class Cv4bTesting extends LinearOpMode {
	public static Positions POSITION = Positions.BASE;
	public static double POSITION_V4B = 0;
	public static double POSITION_COAX = 0;
	public static String SERVOL = "cv4bLeftServo"; 
	public static String SERVOR = "cv4bRightServo"; 
	public static String SERVOCOAX = "cv4bCoaxialServo"; 
	private Servo cv4bLeftServo;
	private Servo cv4bRightServo;
	private Servo cv4bCoaxialServo;

	// POSITIONS (V4B, COAX)
	// Base: 0.14, 0.2
	// Specimen Grab: 0.78, 0.5
	// Pre-deposit: 0.68, 0.35
	// Dump: 0.68, 0.8


	private enum Positions {
		BASE,
		TRANSFER,
		SPECIMEN_GRAB,
		PRE_DEPOSIT,
		DUMP,
		MANUAL
	} 
	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		cv4bLeftServo = hardwareMap.get(Servo.class, "cv4bLeftServo");
		cv4bRightServo = hardwareMap.get(Servo.class, "cv4bRightServo");
		cv4bLeftServo.setDirection(Servo.Direction.FORWARD);
		cv4bRightServo.setDirection(Servo.Direction.REVERSE);
		cv4bCoaxialServo = hardwareMap.get(Servo.class, "cv4bCoaxialServo");
		cv4bCoaxialServo.setDirection(Servo.Direction.REVERSE);

		waitForStart();

		while (opModeIsActive()) {
			switch (POSITION) {
				case BASE:
					setCV4BPosition(0.14, 0.2);
					break;
				case SPECIMEN_GRAB:
					setCV4BPosition(0.78, 0.5);
					break;
				case TRANSFER:
					setCV4BPosition(0.33, 0);
					break;
				case PRE_DEPOSIT:
					setCV4BPosition(0.68, 0.35);
					break;
				case DUMP:
					setCV4BPosition(0.68, 0.8);
					break;
				case MANUAL:
					setCV4BPosition(POSITION_V4B, POSITION_COAX);
					break;
			}

			telemetry.addData("Position L", cv4bLeftServo.getPosition());
			telemetry.addData("Position R", cv4bRightServo.getPosition());
			telemetry.addData("Position C", cv4bCoaxialServo.getPosition());
			telemetry.update();

		}
	}

	public void setCV4BPosition(double v4bRot, double coaxialRot) {
		// POSITIONS (V4B, COAX)
		// Base: 0.14, 0.2
		// Specimen Grab: 0.78, 0.5
		// Pre-deposit: 0.68, 0.35
		// Dump: 0.68, 0.8
		cv4bLeftServo.setPosition(v4bRot);
		cv4bRightServo.setPosition(v4bRot);
		cv4bCoaxialServo.setPosition(coaxialRot);
	}
}