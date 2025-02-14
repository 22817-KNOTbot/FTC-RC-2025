package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@TeleOp(name="Reset Encoders", group="Z")
public class Reset extends LinearOpMode {
	public static boolean INTAKE = true;
	public static boolean DEPOSIT = true;

	@Override
	public void runOpMode() {
		new Intake(hardwareMap, INTAKE);
		new Slides(hardwareMap, DEPOSIT);
	}
}