package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Robot Inspection", group="Debug")
public class RobotInspection extends LinearOpMode {
	public static double BUCKET_POSITION = 1;
	public static Cv4bPosition CV4B_POSITION = Cv4bPosition.SPECIMEN_GRAB;
	public static int SLIDE_POSITION = 3800;
	public static int INTAKE_POSITION = 1800;
	private static boolean extended = false;
	private Servo cv4bLeftServo;
	private Servo cv4bRightServo;
	private Servo cv4bCoaxialServo;
	public DcMotor slideMotorLeft;
	public DcMotor slideMotorRight;

	private enum Cv4bPosition {
		BASE,
		TRANSFER,
		PRE_DEPOSIT,
		DUMP,
		SPECIMEN_GRAB
	}

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		// Bucket: 0 = up, 1 = down
		Servo flipServo = hardwareMap.get(Servo.class, "flipServo");

		cv4bLeftServo = hardwareMap.get(Servo.class, "cv4bLeftServo");
		cv4bRightServo = hardwareMap.get(Servo.class, "cv4bRightServo");
		cv4bLeftServo.setDirection(Servo.Direction.FORWARD);
		cv4bRightServo.setDirection(Servo.Direction.REVERSE);
		cv4bCoaxialServo = hardwareMap.get(Servo.class, "cv4bCoaxialServo");
		cv4bCoaxialServo.setDirection(Servo.Direction.REVERSE);

		slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
		slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
		slideMotorLeft.setPower(1);
		slideMotorRight.setPower(1);
		slideMotorLeft.setTargetPosition(0);
		slideMotorRight.setTargetPosition(0);
		slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		DcMotor intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
		intakeSlides.setPower(1);
		intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		intakeSlides.setTargetPosition(0);
		intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		waitForStart();

		while (opModeIsActive()) {
			if (!extended) {
				flipServo.setPosition(0);
				setCV4BPosition(Cv4bPosition.BASE);
				setSlidePosition(0);
				intakeSlides.setTargetPosition(0);
			} else {
				flipServo.setPosition(BUCKET_POSITION);
				setCV4BPosition(CV4B_POSITION);
				setSlidePosition(SLIDE_POSITION);
				intakeSlides.setTargetPosition(INTAKE_POSITION);
			}

			if (gamepad1.left_bumper) {
				extended = false;
			} else if (gamepad1.right_bumper) {
				extended = true;
			}
		}
	}

	public void setCV4BPosition(Cv4bPosition position) {
		switch (position) {
			case BASE:
				setCV4BPosition(0.14, 0.2);
				break;
			case TRANSFER:
				setCV4BPosition(0.3, 0.1);
				break;
			case PRE_DEPOSIT:
				setCV4BPosition(0.68, 0.35);
				break;
			case DUMP:
				setCV4BPosition(0.68, 0.8);
				break;
			case SPECIMEN_GRAB:
				setCV4BPosition(0.78, 0.5);
				break;
		}
	}

	public void setCV4BPosition(double v4bRot, double coaxialRot) {
		cv4bLeftServo.setPosition(v4bRot);
		cv4bRightServo.setPosition(v4bRot);
		cv4bCoaxialServo.setPosition(coaxialRot);
	}

	public void setSlidePosition(int targetPos) {
		// Max 4200
		slideMotorLeft.setTargetPosition(targetPos);
		slideMotorRight.setTargetPosition(targetPos);
	}
}