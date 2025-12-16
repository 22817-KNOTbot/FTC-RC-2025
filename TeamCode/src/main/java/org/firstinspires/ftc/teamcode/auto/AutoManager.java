package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoState;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.StartAutoState;
import org.firstinspires.ftc.teamcode.util.Alliance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoManager {
	private Alliance alliance;
	private AutoComponents components;

	private boolean pathFinished = true;
	private List<PathChain> pathList;
	private int currentPathState = 0;

	public AutoManager(Alliance alliance) {
		this.alliance = alliance;
		this.components = new AutoComponents(alliance);
	}

	public void initialize(HardwareMap hardwareMap, Follower follower, StartAutoState startAutoState, List<AutoAction> autoActions) {
		pathList = new ArrayList<>();
		Pose previousPose = startAutoState.getStartPose(alliance);
		for (AutoAction autoAction : autoActions) {
			PathChain newPathChain = autoAction.getPathChain(follower, previousPose);
			pathList.add(newPathChain);
			previousPose = newPathChain.endPose();
		}
	}

	public StartAutoState[] getStartingStates() {
		return new StartAutoState[] {
			components.new StartLowState(),
			components.new StartUpState(),
		};
	}

	public void updatePathFollowing(Follower follower) {
		if (pathFinished) {
			if (currentPathState + 1 < pathList.size()) {
				currentPathState++;
				follower.followPath(pathList.get(currentPathState), true);
			}
		}
	}
}
