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
	public static Intake.Positions BUCKET_POSITION = Intake.Positions.TRANSFER;
	public static double BUCKET_POSITION_MANUAL_DRIVE = -1;
	public static double BUCKET_POSITION_MANUAL_COAX = -1;
	public static double SLIDE_POWER = 1;
	public static Intake.Positions SLIDE_POSITION = Intake.Positions.TRANSFER;
	public static int SLIDE_POSITION_MANUAL = -1;
	public static double INTAKE_POWER = 0;
	public Intake intake;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		intake = new Intake(hardwareMap, true);

		waitForStart();

		while (opModeIsActive()) {
			intake.setPower(INTAKE_POWER);
			// intake.setSlidePower(SLIDE_POWER);
			if (BUCKET_POSITION_MANUAL_DRIVE == -1 || BUCKET_POSITION_MANUAL_COAX == -1) {
				intake.setBucketPosition(BUCKET_POSITION);
			} else {
				intake.setBucketPosition(BUCKET_POSITION_MANUAL_DRIVE, BUCKET_POSITION_MANUAL_COAX);
			}
			// if (SLIDE_POSITION_MANUAL == -1) {
			// 	intake.setSlidePosition(SLIDE_POSITION);
			// } else {
			// 	intake.setSlidePosition(SLIDE_POSITION_MANUAL);
			// }
		}
	}
}