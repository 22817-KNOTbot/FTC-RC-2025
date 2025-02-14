package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
	static {
		FollowerConstants.localizers = Localizers.THREE_WHEEL;

		FollowerConstants.leftFrontMotorName = "leftFront";
		FollowerConstants.leftRearMotorName = "leftBack";
		FollowerConstants.rightFrontMotorName = "rightFront";
		FollowerConstants.rightRearMotorName = "rightBack";

		FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
		FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
		FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
		FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

		FollowerConstants.mass = 14.5;

		FollowerConstants.xMovement = 74.93937809584149;
		FollowerConstants.yMovement = 53.89778914777473;

		FollowerConstants.forwardZeroPowerAcceleration = -31.3286635177;
		FollowerConstants.lateralZeroPowerAcceleration = -83.31627994322986;

		FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.54,0.000001,0.0434375,0);
		FollowerConstants.useSecondaryTranslationalPID = false;
		FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

		FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
		FollowerConstants.useSecondaryHeadingPID = false;
		FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

		FollowerConstants.drivePIDFCoefficients.setCoefficients(0.022,0,0.000001,0.6,0);
		FollowerConstants.driveKalmanFilterParameters.modelCovariance = 6.7;
		FollowerConstants.driveKalmanFilterParameters.dataCovariance = 1;
		FollowerConstants.useSecondaryDrivePID = false;
		FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

		FollowerConstants.zeroPowerAccelerationMultiplier = 4.3;
		FollowerConstants.centripetalScaling = 0.0005;

		FollowerConstants.pathEndTimeoutConstraint = 500;
		FollowerConstants.pathEndTValueConstraint = 0.995;
		FollowerConstants.pathEndVelocityConstraint = 0.1;
		FollowerConstants.pathEndTranslationalConstraint = 0.1;
		FollowerConstants.pathEndHeadingConstraint = 0.007;
	}
}