package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;


public class MecanumDrive {

	public void runOpMode(){

		waitForStart();
		follower.startTeleopDrive();

		while (opModeIsActive){
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
		}

	}

}


