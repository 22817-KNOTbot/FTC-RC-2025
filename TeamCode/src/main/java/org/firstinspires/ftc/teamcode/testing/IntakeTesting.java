package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name="Intake testing", group="Debug")
public class IntakeTesting extends LinearOpMode {
	public static Intake.Positions INTAKE_POSITION = Intake.Positions.TRANSFER;
	public static boolean INTAKE_POSITION_MANUAL = false;
	public static double INTAKE_POSITION_DRIVE = 0;
	public static double INTAKE_POSITION_COAX = 0;
	public static double SLIDE_POWER = 1;
	public static Intake.Positions SLIDE_POSITION = Intake.Positions.TRANSFER;
	public static int SLIDE_POSITION_MANUAL = -1;
	public static double INTAKE_WRIST_DEG = 0;
	public static boolean INTAKE_CLAW_CLOSED = false;
	public Intake intake;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		intake = new Intake(hardwareMap, true);

		waitForStart();

		while (opModeIsActive()) {
			intake.setSlidePower(SLIDE_POWER);
			if (INTAKE_POSITION_MANUAL) {
				intake.setIntakePosition(INTAKE_POSITION_DRIVE, INTAKE_POSITION_COAX);
			} else {
				intake.setIntakePosition(INTAKE_POSITION);
			}
			if (SLIDE_POWER != 0) {
				if (SLIDE_POSITION_MANUAL == -1) {
					intake.setSlidePosition(SLIDE_POSITION);
				} else {
					intake.setSlidePosition(SLIDE_POSITION_MANUAL);
				}
			}
			intake.setWristRotation(INTAKE_WRIST * Intake.WRIST_VALUE_PER_DEG);
			if (INTAKE_CLAW_CLOSED) intake.closeClaw(); else intake.openClaw();
		}
	}
}