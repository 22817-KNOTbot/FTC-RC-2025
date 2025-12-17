package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoActionCommand;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoState;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.StartAutoState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoManager {
	private Alliance alliance;
	private AutoComponents components;

	private Follower follower;
	private Automations automationHandler;
	private List<AutoAction> autoActions;

	private boolean initialized = false;
	private boolean startedFollowingPath;
	private boolean stateFinished;
	private List<PathChain> pathList;
	private int currentState = 0;

	public AutoManager(Alliance alliance) {
		this.alliance = alliance;
		this.components = new AutoComponents(alliance);
	}

	public void initialize(HardwareMap hardwareMap, StartAutoState startAutoState, List<AutoAction> autoActions) {
		if (initialized)
			return;
		this.follower = Constants.createFollower(hardwareMap);
		this.follower.setStartingPose(startAutoState.getStartPose());
		this.automationHandler = new Automations(hardwareMap, alliance, true);
		this.autoActions = autoActions;
		this.stateFinished = false;

		this.pathList = new ArrayList<>();
		Pose previousPose = startAutoState.getStartPose();
		for (AutoAction autoAction : autoActions) {
			PathChain newPathChain = autoAction.getPathChain(follower, previousPose);
			this.pathList.add(newPathChain);
			previousPose = newPathChain.endPose();
		}
	}

	public StartAutoState[] getStartingStates() {
		return new StartAutoState[] {
				components.new StartLowState(),
				components.new StartUpState(),
		};
	}

	public void update() {
		if (currentState < 0)
			return;
		updateCommands();
		updatePathFollowing();

		if (stateFinished) {
			if (currentState + 1 < autoActions.size()) {
				setState(currentState + 1);
			} else {
				setState(-1);
			}
		}
	}

	public void updateCommands() {
		if (stateFinished)
			return;
		AutoAction currentAction = autoActions.get(currentState);
		AutoActionCommand currentCommand = currentAction.getActionCommand();
		if (currentCommand != null) {
			stateFinished = currentCommand.run(follower, automationHandler);
		}
	}

	public void updatePathFollowing() {
		if (!startedFollowingPath) {
			follower.followPath(pathList.get(currentState), true);
			startedFollowingPath = true;
		}
	}

	private void setState(int state) {
		currentState = state;
		startedFollowingPath = false;
		stateFinished = false;
	}

	public List<AutoAction> getAutoActions() {
		return new ArrayList<>(autoActions);
	}

	public void showTelemetry(TelemetryManager telemetryManager) {
		telemetryManager.addData("State", currentState);
		telemetryManager.addData("Current action", autoActions.get(currentState).getNameString());
	}

	public void showAutomationsTelemetry(TelemetryManager telemetryManager) {
		automationHandler.showTelemetry(telemetryManager);
	}

	public void showFollowerTelemetry(TelemetryManager telemetryManager) {
		String[] debugLines = null;
		try {
			debugLines = follower.debug();
		} catch (Exception e) {
			telemetryManager.addLine("Failed to retrieve debug string!");
		}
		if (debugLines != null) {
			for (String line : debugLines) {
				telemetryManager.addLine(line);
			}
		}

	}
}
