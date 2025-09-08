package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;




public class MecanumDrive {


	private Pose holdPose = new Pose(0, 0, 0);
	private boolean holdingPose;
    private Follower follower;

    public MecanumDrive(HardwareMap hardwareMap){
        follower = Constants.createFollower(hardwareMap);
    }

	public void mecanumDrive(float forward, float lateral, float rotation){
		
		follower.startTeleopDrive();

        follower.update();

        double denominator = Math.max(Math.abs(forward) + Math.abs(lateral) + Math.abs(rotation), 1);

        follower.setTeleOpDrive(
            forward / denominator,
            lateral / denominator,
            rotation / denominator,
            false); 

        // if (!holdingPose && gamepad1.left_bumper) {
        //     holdPose = follower.getPose();
        //     follower.holdPoint(holdPose);
        //     holdingPose = true;
        // } else if (holdingPose && !gamepad1.left_bumper) {
        //     follower.breakFollowing();
        //     follower.startTeleopDrive();
        //     holdingPose = false;
        // }

	}

    public void lockingMecanum(boolean enabled){
        if (enabled){
            holdPose = follower.getPose();
             follower.holdPoint(holdPose);
             holdingPose = true;
        }else{
            follower.breakFollowing();
             follower.startTeleopDrive();
             holdingPose = false;
        }
    }

}


