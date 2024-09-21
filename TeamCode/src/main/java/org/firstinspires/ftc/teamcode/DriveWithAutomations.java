package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Drive With Automations", group="Automations")
public class DriveWithAutomations extends LinearOpMode {

	private ElapsedTime runtime = new ElapsedTime();
	private DcMotor leftFrontDrive = null;
	private DcMotor leftBackDrive = null;
	private DcMotor rightFrontDrive = null;
	private DcMotor rightBackDrive = null;
	private boolean automationRunning = false;
	private String automationName = "";
	private Automations automationHandler = new Automations();

	@Override
	public void runOpMode() {
		leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
		leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
		rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
		rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();
		runtime.reset();

		while (opModeIsActive()) {
			// Drive function
			double max;

			double axial   = -gamepad1.left_stick_y;
			double lateral =  gamepad1.left_stick_x;
			double yaw     =  gamepad1.right_stick_x;

			double leftFrontPower  = axial + lateral + yaw;
			double rightFrontPower = axial - lateral - yaw * 0.8;
			double leftBackPower   = axial - lateral + yaw;
			double rightBackPower  = axial + lateral - yaw;

			max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
			max = Math.max(max, Math.abs(leftBackPower));
			max = Math.max(max, Math.abs(rightBackPower));

			if (max > 1.0) {
				leftFrontPower  /= max;
				rightFrontPower /= max;
				leftBackPower   /= max;
				rightBackPower  /= max;
			}

			leftFrontDrive.setPower(leftFrontPower);
			rightFrontDrive.setPower(rightFrontPower);
			leftBackDrive.setPower(leftBackPower);
			rightBackDrive.setPower(rightBackPower);

			// Automation triggering
			if (gamepad1.left_bumper) {
				
			}

			// Telemtry
			telemetry.addData("Status", "Run Time: " + runtime.toString());
			telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
			telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
			telemetry.addData("Automation", automationRunning ? automationName : "None");
			telemetry.update();

		}
	}}
