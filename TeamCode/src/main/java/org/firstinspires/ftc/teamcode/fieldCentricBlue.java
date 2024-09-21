package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

@TeleOp(name="Field Centric Teleop: Blue", group="Field Centric")
public class fieldCentricBlue extends LinearOpMode {

    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    private String automationName;
	private Automations automationHandler = new Automations();

    
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {        
        //declare motors
        //match ID's to config
        DcMotor frontLeftDrive  = hardwareMap.DcMotor.get("frontLeftDrive");
        DcMotor backLeftDrive  = hardwareMap.DcMotor.get("backLeftDrive");
        DcMotor frontRightDrive  = hardwareMap.DcMotor.get("frontRightDrive");
        DcMotor backRightDrive  = hardwareMap.DcMotor.get("backRightDrive");

        //reverse left motors (depends on robot)
        frontLeftDrive.setDirection(DcMotorSimple.Direction.Reverse);
        backLeftDrive.setDirection(DcMotorSimple.Direction.Reverse);
        
        IMU imu = hardwareMap.get(IMU.class, "imu"); //get IMU from hardware map
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.Up, //depends on robot (control hub configuration)
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)); //depends on robot (control hub configuration)
        imu.initialize(parameters); //init parameters

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_y;
            double rx = -gamepad1.left_stick_y;

            //reset heading is need
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.Radians);

            rotX *= 1.1; // Counteract imperfect strafing

            //rotate by robot's rotation to maintain field-centric drive (90 degrees)
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = y * Math.sin(-heading) + y * Math.cos(-heading);

            //basic mecanum code for motor power
            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            //set motor power to motor power variables
            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            if (!automationHandler.running()) {
                if (gamepad1.left_bumper) {
                    automationName = "Intake";
                    // TODO: Alliance
                    automationHandler.intake(hardwareMap, true);
                }
            }

            telemetry.addData("Automation", automationRunning ? automationName : "None");
			telemetry.update();
        }
    }
}