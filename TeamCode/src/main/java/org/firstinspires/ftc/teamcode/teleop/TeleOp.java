package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.List;

@Configurable
public class TeleOp extends LinearOpMode {
	public static boolean DEBUG = false;

	private ElapsedTime runtime = new ElapsedTime();
	private Automations automationHandler;
	private MecanumDrive mecanumDrive;

	@Override
	public void runOpMode() {
		if (gamepad1.right_bumper)
			DEBUG = true;
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		// Bulk read
		List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
		}

		automationHandler = new Automations(hardwareMap, DEBUG);
		mecanumDrive = new MecanumDrive(hardwareMap);

		waitForStart();

		mecanumDrive.initialize();
		runtime.reset();

		while (opModeIsActive()) {
			// IMPORTANT: Cache must be cleared every loop to prevent stale data
			for (LynxModule hub : allHubs) {
				hub.clearBulkCache();
			}

			mecanumDrive.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

			mecanumDrive.lockingMecanum(gamepad1.left_bumper);

			switch (automationHandler.automationState) {
				case ABORT:
					break;
				case IDLE:
					break;
				case INTAKE_OPEN:
					break;
				case INTAKE_OFF:
					break;
				case TRANSFER:
					break;
				case ARTIFACT_LOADED:
					break;
				case ARTIFACT_EJECT_WAIT:
					break;
				case ARTIFACT_EJECT:
					break;
			}

			if (gamepad1.aWasPressed()) {
				automationHandler.grab();
			}
			if (gamepad1.bWasPressed()) {
				automationHandler.putIntoTurret();
			}
			if (gamepad1.xWasPressed()) {
				automationHandler.fireTurret();
			}
			if (gamepad1.yWasPressed()) {

			}
		}

		if (gamepad1.back || gamepad2.back) {
			automationHandler.automationState = Automations.State.ABORT;
		} else if (gamepad1.start || gamepad2.start) {
			mecanumDrive.resetPose();
		}

		telemetry.addData("Time", runtime.time());
		telemetry.addData("State", automationHandler.automationState);
		if (!automationHandler.colourSensorResponding()) {
			telemetry.addLine("********************");
			telemetry.addLine("WARNING: COLOUR SENSOR");
			telemetry.addLine("IS NOT RESPONDING");
			telemetry.addLine("********************");
		}
		if (DEBUG) {
			Pose pose = mecanumDrive.getPose();
			Pose holdPose = mecanumDrive.getHoldPose();

			telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));

			telemetry.addData("Position X", pose.getX());
			telemetry.addData("Position Y", pose.getY());

			telemetry.addData("Hold Position X", holdPose.getX());
			telemetry.addData("Hold Position Y", holdPose.getY());
			automationHandler.showTelemetry(telemetry);
		}
		telemetry.update();

	}

}
