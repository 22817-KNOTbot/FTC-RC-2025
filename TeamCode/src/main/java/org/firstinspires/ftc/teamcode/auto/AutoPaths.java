package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class AutoPaths {
	public static class Red {
		// Close Paths
		public static PathChain getBottomApproach(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierCurve(startingPose,
            						new Pose(90.000, 35.000),
            						new Pose(100.000, 35.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
        			.build();
		}

		public static PathChain getMiddleApproach(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(100.000, 58.000)))
					.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
					.build();
		}

		public static PathChain getTopApproach(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(100.000, 84.000)))
					.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
					.build();
		}
		
		public static PathChain getIntake(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, startingPose.withX(startingPose.getX() + 25)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();
		}
			
		public static PathChain getIntakePreparedSet(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, startingPose.withX(startingPose.getX() + 35)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();
		}

		public static PathChain getGateIntake(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
        					new BezierCurve(
									startingPose,
									new Pose(90.000, 69.000),
									new Pose(128.000, 69.000)
							))
					.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
					.addPath(
						new BezierLine(startingPose, new Pose(131.000, 60.000))
					)
					.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
					.build();
		}

		public static PathChain getUpLaunch(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(84.000, 84.000)))
					.setTangentHeadingInterpolation()
					.setReversed()
					.build();
		}

		public static PathChain getExitUpperShootingZone(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(84.000, 105.000)))
					.setConstantHeadingInterpolation(Math.toRadians(340))
					.build();
		}

		// Other Paths

		public static PathChain getPushGate(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierCurve(
									startingPose,
									new Pose(96.000, 56.000),
									new Pose(129.000, 69.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();
		}

		public static PathChain getLowLaunch(Follower follower, Pose startingPose) {
			if (startingPose.getY() >= 40) {
				return follower.pathBuilder()
						.addPath(
								new BezierCurve(startingPose,
										startingPose.withX(96),
										new Pose(96.000, 11.000)))
						.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
						.build();
			} else {
				return follower.pathBuilder()
						.addPath(
								new BezierLine(startingPose, new Pose(96.000, 11.000)))
						.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
						.build();
			}
		}

		public static PathChain getLowLaunchCurved(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierCurve(startingPose,
									startingPose.withX(96),
									new Pose(96.000, 11.000)))
					.setLinearHeadingInterpolation(startingPose.getHeading(), Math.toRadians(0))
					.build();
		}

		public static PathChain getExitLowShootingZone(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(96.000, 30.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();
		}

		public static PathChain getExitUpShootingZone(Follower follower, Pose startingPose) {
			return follower.pathBuilder()
					.addPath(
							new BezierLine(startingPose, new Pose(96.000, 70.000)))
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();
		}
	}

	public static class Blue {
		public static PathChain getBottomApproach(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getBottomApproach(follower, startingPose.mirror()), follower);
		}

		public static PathChain getMiddleApproach(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getMiddleApproach(follower, startingPose.mirror()), follower);
		}

		public static PathChain getPushGate(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getPushGate(follower, startingPose.mirror()), follower);
		}

		public static PathChain getTopApproach(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getTopApproach(follower, startingPose.mirror()), follower);
		}

		public static PathChain getIntakePreparedSet(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getIntakePreparedSet(follower, startingPose.mirror()), follower);
		}

		public static PathChain getExitUpperShootingZone(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getExitUpperShootingZone(follower, startingPose.mirror()), follower);
		}

		public static PathChain getGateIntake(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getGateIntake(follower, startingPose.mirror()), follower);
		}


		public static PathChain getLowLaunch(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getLowLaunch(follower, startingPose.mirror()), follower);
		}

		public static PathChain getUpLaunch(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getUpLaunch(follower, startingPose.mirror()), follower);
		}

		public static PathChain getExitLowShootingZone(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getExitLowShootingZone(follower, startingPose.mirror()), follower);
		}

		public static PathChain getExitUpShootingZone(Follower follower, Pose startingPose) {
			return mirrorPathChain(Red.getExitUpShootingZone(follower, startingPose.mirror()), follower);
		}
	}

	public static PathChain mirrorPathChain(PathChain pathChain, Follower follower) {
		PathBuilder newPathBuilder = follower.pathBuilder();
		for (int i = 0; i < pathChain.size(); i++) {
			Path path = pathChain.getPath(i);
			List<Pose> controlPoints = path.getControlPoints();
			List<Pose> newControlPoints = new ArrayList<>();
			for (Pose controlPoint : controlPoints) {
				newControlPoints.add(controlPoint.mirror());
			}

			Path newPath = new Path(new BezierCurve(newControlPoints, path.getConstraints()), path.getConstraints());
			newPath.setHeadingInterpolation(closestPoint -> MathFunctions
					.normalizeAngle(Math.PI - path.getHeadingInterpolator().interpolate(closestPoint)));
			newPathBuilder.addPath(newPath);
		}

		return newPathBuilder.build();
	}
}
