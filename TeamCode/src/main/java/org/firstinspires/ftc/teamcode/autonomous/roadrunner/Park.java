package org.firstinspires.ftc.teamcode.autonomous.roadrunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Autonomous(name = "Park Auto", group = "Backup")
public class Park extends LinearOpMode {
    	
	@Override
	public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, -61, Math.toRadians(90)));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(24, -61, Math.toRadians(90)))
            .strafeTo(new Vector2d(48, -61));

		waitForStart();

		Actions.runBlocking(
            park.build()
		);

        // Store pose for future use
		OpModeStorage.pose = drive.pose;
	}
}
