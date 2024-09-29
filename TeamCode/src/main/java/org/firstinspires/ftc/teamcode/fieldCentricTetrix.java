package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

@TeleOp(name="Tetrix: Field Centric Teleop", group="Tetrix")
public class fieldCentricTetrix extends LinearOpMode {

	public DcMotor frontLeftDrive;
	public DcMotor backLeftDrive;
	public DcMotor frontRightDrive;
	public DcMotor backRightDrive;
	private String automationName;	
	
	IMU imu;
	
	@Override
	public void runOpMode() {        
		DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
		DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
		DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
		DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

		frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
		
		IMU imu = hardwareMap.get(IMU.class, "imu");
		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
			RevHubOrientationOnRobot.LogoFacingDirection.UP,
			RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		imu.initialize(parameters);

		waitForStart();

		while (opModeIsActive()) {
			double y = -gamepad1.left_stick_y;
			double x = gamepad1.left_stick_x;
			double rx = gamepad1.right_stick_x;

			double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

			double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
			double rotY = y * Math.sin(-heading) + y * Math.cos(-heading);
			
			rotX *= 1.1;
			
			double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);
			double frontLeftPower = (rotY + rotX + rx) / denominator;
			double backLeftPower = (rotY - rotX + rx) / denominator;
			double frontRightPower = (rotY - rotX - rx) / denominator;
			double backRightPower = (rotY + rotX - rx) / denominator;

			frontLeftDrive.setPower(frontLeftPower);
			backLeftDrive.setPower(backLeftPower);
			frontRightDrive.setPower(frontRightPower);
			backRightDrive.setPower(backRightPower);
			
			if (gamepad2.start) {
				imu.resetYaw();
			}	
			
			telemetry.addData("frontLeftPower", frontLeftPower);
			telemetry.addData("frontRightPower", frontRightPower);
			telemetry.addData("backLeftPower", backLeftPower);
			telemetry.addData("backRightPower", backRightPower);
			telemetry.update();
		}
	}
}