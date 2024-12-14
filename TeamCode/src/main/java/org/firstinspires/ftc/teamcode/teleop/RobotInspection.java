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

import org.firstinspires.ftc.teamcode.subsystems.CV4B;

@Config
@TeleOp(name="Robot Inspection", group="Debug")
public class RobotInspection extends LinearOpMode {
	public static CV4B.Positions CV4B_POSITION = CV4B.Positions.SPECIMEN_GRAB;
	public static int SLIDE_POSITION = 4100;
	public static int INTAKE_POSITION = 800;
	private static boolean extended = false;
	public DcMotor slideMotorLeft;
	public DcMotor slideMotorRight;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		cv4b = new CV4B(harwareMap);

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
				cv4b.setPosition(CV4B.Positions.TRANSFER);
				setSlidePosition(0);
				intakeSlides.setTargetPosition(0);
			} else {
				cv4b.setPosition(CV4B_POSITION);
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

	public void setSlidePosition(int targetPos) {
		// Max 4200
		slideMotorLeft.setTargetPosition(targetPos);
		slideMotorRight.setTargetPosition(targetPos);
	}
}