package org.firstinspires.ftc.teamcode;

import java.util.List;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.config.Config;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

@Config
@TeleOp(name="Field Centric Teleop: Blue", group="Field Centric")
public class fieldCentricBlue extends LinearOpMode {
	public static boolean DEBUG = false;
	public DcMotor frontLeftDrive;
	public DcMotor backLeftDrive;
	public DcMotor frontRightDrive;
	public DcMotor backRightDrive;
	private String automationName;
	private Automations automationHandler; 
	private Automations.Basket targetBasket;

	private boolean postDeposit = false;
	
	IMU imu;
	
	@Override
	public void runOpMode() {        
		automationHandler = new Automations(hardwareMap, DEBUG);
		targetBasket = Automations.Basket.HIGH;

		frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
		backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
		frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
		backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

		frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
		backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
		
		IMU imu = hardwareMap.get(IMU.class, "imu");
		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
			RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
			RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		imu.initialize(parameters);
		imu.resetYaw();

		// Bulk read. If max performance is needed use MANUAL
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}
		// ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, (double) 70/105);

		waitForStart();

		while (opModeIsActive()) {
			double y = -gamepad1.left_stick_y;
			double x = gamepad1.left_stick_x;
			double rx = gamepad1.right_stick_x;

			double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
			// Twist2dDual<Time> twist = localizer.update();
			// heading = twist.value().heading;

			double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
			double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
			
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

			switch (Automations.automationState) {
				case ABORT:
					automationName = "ABORTING";
					automationHandler.abort();
					break;
				case IDLE:
					if (gamepad1.right_bumper) {
						automationName = "Intake";
						automationHandler.intakeInit();
					} else if (gamepad1.right_trigger > 0.9) {
						automationName = "Ascend";
						automationHandler.ascendInit();
					} else if (gamepad1.a) {
						automationName = "Hang specimen";
						automationHandler.hangSpecimen();
					} else if (gamepad1.b && !postDeposit) {
						automationName = "Deposit sample";
						automationHandler.depositInit(targetBasket);
					} else if (gamepad1.x) {
						automationName = "Lower slides";
						automationHandler.setSlidePosition(0);
						postDeposit = false;
					} else {
						if (automationHandler.slideMotor1.getCurrentPosition() < 5) {
							automationHandler.slideMotor1.setPower(0);
						}
						if (automationHandler.slideMotor2.getCurrentPosition() < 5) {
							automationHandler.slideMotor2.setPower(0);
						}
					}

					break;
				case INTAKE_WAIT:
					automationHandler.intakeWait();
					break;
				case INTAKE_FILLED:
					automationHandler.intakeFilled(Automations.Alliance.BLUE, true);
					break;
				case INTAKE_DUMPING:
					automationHandler.intakeDumping();
					break;
				case TRANSFER:
					automationHandler.transferInit();
					break;
				case TRANSFER_WAIT:
					automationHandler.transferWait();
					break;
				case DEPOSIT_EXTENDING:
					automationHandler.depositExtending();
					break;
				case DEPOSIT_EXTENDED:
					if (gamepad1.b) {
						automationHandler.depositSample();
						postDeposit = true;
					}
					break;
				case ASCEND_LOW_EXTENDED:
					if (gamepad1.right_trigger > 0.9) {
						automationHandler.ascendLowRetract();
					}
			}

			if (gamepad2.back) {
				Automations.automationState = Automations.State.ABORT;
			} else if (gamepad2.start) {
				imu.resetYaw();
			} else if (gamepad2.a) {
				targetBasket = targetBasket == Automations.Basket.HIGH ? Automations.Basket.LOW : Automations.Basket.HIGH;
			}

			telemetry.addData("State", Automations.automationState);
			if (DEBUG) {
				telemetry.addData("Heading", Math.toDegrees(heading));
				telemetry.addData("frontLeftPower", frontLeftPower);
				telemetry.addData("frontRightPower", frontRightPower);
				telemetry.addData("backLeftPower", backLeftPower);
				telemetry.addData("backRightPower", backRightPower);
				automationHandler.updateDashboardTelemetry();
			}
			telemetry.update();
		}
	}
}