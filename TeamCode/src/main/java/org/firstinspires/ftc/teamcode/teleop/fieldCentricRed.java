package org.firstinspires.ftc.teamcode.teleop;

import java.util.List;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Automations;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.ControlTheory;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.GamepadStorage;

@Config
@TeleOp(name = "Field Centric Teleop: Red", group = "AAA Field Centric")
public class fieldCentricRed extends LinearOpMode {
	public static boolean DEBUG = false;
	private ElapsedTime runtime = new ElapsedTime();
	private Automations automationHandler;
	private Automations.Basket targetBasket;

	private Follower follower;
	private Pose pose;

	private Pose holdPose = new Pose(0, 0, 0);
	private boolean holdingPose;

	@Override
	public void runOpMode() {
		if (gamepad1.right_bumper)
			DEBUG = true;
		boolean buttonPressed = false;
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		GamepadStorage.gamepad1 = gamepad1;
		GamepadStorage.gamepad2 = gamepad2;
		automationHandler = new Automations(hardwareMap,
				OpModeStorage.mode == null ? Automations.Modes.SPECIMEN : OpModeStorage.mode, DEBUG);
		targetBasket = Automations.Basket.HIGH;

		if (OpModeStorage.x > Double.NEGATIVE_INFINITY
				&& OpModeStorage.y > Double.NEGATIVE_INFINITY
				&& OpModeStorage.heading > Double.NEGATIVE_INFINITY) {
			pose = new Pose(OpModeStorage.x, OpModeStorage.y, OpModeStorage.heading);
		} else {
			pose = new Pose(0, 0, 0);
		}
		Constants.setConstants(FConstants.class, LConstants.class);
		follower = new Follower(hardwareMap);
		follower.setStartingPose(pose);

		// Bulk read
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}

		if (!automationHandler.colourSensorResponding()) {
			automationHandler.vibrateControllers(1000);
			telemetry.addLine("********************");
			telemetry.addLine("WARNING: COLOUR SENSOR");
			telemetry.addLine("IS NOT RESPONDING");
			telemetry.addLine("********************");
		}

		waitForStart();
		follower.startTeleopDrive();
		runtime.reset();

		while (opModeIsActive()) {
			// IMPORTANT: Cache must be cleared every loop to prevent stale data
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
			}

			follower.update();

			double forward = -gamepad1.left_stick_y;
			double lateral = -gamepad1.left_stick_x;
			double rotation = -gamepad1.right_stick_x;

			double denominator = Math.max(Math.abs(forward) + Math.abs(lateral) + Math.abs(rotation), 1);

			follower.setTeleOpMovementVectors(
					forward / denominator,
					lateral / denominator,
					rotation / denominator,
					false);

			if (!holdingPose && gamepad1.left_bumper) {
				holdPose = follower.getPose();
				follower.holdPoint(holdPose);
				holdingPose = true;
			} else if (holdingPose && !gamepad1.left_bumper) {
				follower.breakFollowing();
				follower.startTeleopDrive();
				holdingPose = false;
			}

			switch (automationHandler.automationState) {
				case ABORT:
					automationHandler.abort();
					break;
				case IDLE:
					// Sample mode
					if (automationHandler.getMode() == Automations.Modes.SAMPLE) {
						if (gamepad2.a) {
							automationHandler.intakeInit(Automations.SamplePurpose.SAMPLE);
						} else if (gamepad2.b) {
							automationHandler.depositInit(targetBasket);
						} else if (gamepad2.x) {
							automationHandler.intakeInit(Automations.SamplePurpose.SPECIMEN);
						}

						// Specimen mode
					} else if (automationHandler.getMode() == Automations.Modes.SPECIMEN) {
						if (gamepad2.y) {
							automationHandler.grabSpecimen();
						}
					}

					// Switch modes
					if (gamepad2.dpad_left) {
						automationHandler.setMode(Automations.Modes.SAMPLE);
					} else if (gamepad2.dpad_right) {
						automationHandler.setMode(Automations.Modes.SPECIMEN);
					}

					if (gamepad1.left_trigger > 0.9) {
						automationHandler.ascendInit();
					} else if (gamepad1.right_bumper) {
						automationHandler.retract();
					} else if (automationHandler.getSlideLeftPosition() < 5
							&& automationHandler.getSlideRightPosition() < 5
							&& !automationHandler.getSlideBusy()) {
						automationHandler.setSlidesPower(0);
					}
					break;

				// Sample intake
				case INTAKE_READY:
					automationHandler.intakePosition(gamepad2.left_stick_x, -gamepad2.left_stick_y,
						-follower.getPose().getHeading());
					if ((gamepad2.a || gamepad2.x) && !buttonPressed) {
						automationHandler.intakeGrab();
					}
					break;
				case INTAKE_GRABBING:
					automationHandler.intakeGrabbing();
					break;
				case INTAKE_GRABBED:
					automationHandler.intakeGrabbed(Automations.Alliance.RED, true, gamepad2.right_bumper);
					break;
				case INTAKE_RELEASE:
					automationHandler.intakeRelease();
					break;
				// Transfer
				case TRANSFER:
					automationHandler.transferInit();
					break;
				case TRANSFER_WAIT:
					automationHandler.transferWait();
					break;
				case TRANSFERRING:
					automationHandler.transferring();
					break;
				// Deposit
				case TRANSFERRED:
					if (gamepad2.b) {
						automationHandler.depositInit(targetBasket);
					}
					break;
				case DEPOSIT_EXTENDING:
					automationHandler.depositExtending();
					break;
				case DEPOSIT_EXTENDED:
					if (gamepad2.b && !buttonPressed) {
						automationHandler.depositSample();
					}
					break;
				case DEPOSITED:
					if (gamepad1.right_bumper) {
						automationHandler.resetDeposit();
					}
					break;

				// Specimens
				// Give to HP
				case NO_TRANSFER:
					automationHandler.noTransfer();
					break;
				case SAMPLE_LOADED:
					if (gamepad2.x) {
						automationHandler.sampleEject();
					}
					break;
				case SAMPLE_EJECT_WAIT:
					automationHandler.sampleEjectWait();
				case SAMPLE_EJECTED:
					if (gamepad1.right_bumper) {
						automationHandler.resetSampleEject();
					}
					break;
				// Grabbing specimen
				case SPECIMEN_GRABBING:
					automationHandler.grabSpecimenWait();
					break;
				case SPECIMEN_GRABBED:
					if (gamepad1.right_stick_button) {
						automationHandler.hangSpecimen();
					}
					break;
				case SPECIMEN_HANGING:
					automationHandler.hangSpecimenWait();
					break;
				case SPECIMEN_HUNG:
					if (gamepad1.right_bumper) {
						automationHandler.resetSpecimen();
					}
					break;
				// Ascent
				case ASCEND_LOW_EXTENDING:
					automationHandler.ascendLowExtending();
					break;
				case ASCEND_LOW_EXTENDED:
					if (gamepad1.left_trigger > 0.9 && !buttonPressed) {
						automationHandler.ascendLowRetract();
					}
					break;

				// Mode switching
				case SPECIMEN_TO_SAMPLE:
					automationHandler.specimenToSample();
					break;
				case SAMPLE_TO_SPECIMEN:
					automationHandler.sampleToSpecimen();
					break;
				default:
					break;
			}

			if (gamepad1.back || gamepad2.back) {
				automationHandler.automationState = Automations.State.ABORT;
			} else if (gamepad1.start || gamepad2.start) {
				follower.setPose(new Pose(0, 0, 0));
			}
			if (gamepad1.a) {
				targetBasket = targetBasket == Automations.Basket.HIGH ? Automations.Basket.LOW
						: Automations.Basket.HIGH;
			}

			if (gamepad1.dpad_up) {
				CV4B.offset_drive += 0.05;
			} else if (gamepad1.dpad_down) {
				CV4B.offset_drive -= 0.05;
			}

			buttonPressed = gamepad2.b || gamepad2.a || gamepad2.x || gamepad1.left_trigger > 0.9;

			telemetry.addData("Time", runtime.time());
			telemetry.addData("State", automationHandler.automationState);
			telemetry.addData("Mode", automationHandler.getMode());
			if (!automationHandler.colourSensorResponding()) {
				telemetry.addLine("********************");
				telemetry.addLine("WARNING: COLOUR SENSOR");
				telemetry.addLine("IS NOT RESPONDING");
				telemetry.addLine("********************");
			}
			telemetry.addData("Basket", targetBasket);
			if (DEBUG) {
				telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
				telemetry.addData("Forward", forward);
				telemetry.addData("Lateral", lateral);
				telemetry.addData("Rotation", rotation);

				telemetry.addData("Position X", follower.getPose().getX());
				telemetry.addData("Position Y", follower.getPose().getY());

				telemetry.addData("Hold Position X", holdPose.getX());
				telemetry.addData("Hold Position Y", holdPose.getY());
				automationHandler.updateDashboardTelemetry();
			}
			telemetry.update();
		}
	}
}