package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Storage;

@TeleOp
public class ResetOpmode extends LinearOpMode {
	@Override
	public void runOpMode() {
		waitForStart();

		blackboard.remove("alliance");
		blackboard.remove("pose");

		new Storage(hardwareMap, true);

		requestOpModeStop();
	}
}
