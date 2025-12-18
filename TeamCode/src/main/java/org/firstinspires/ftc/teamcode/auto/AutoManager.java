package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoAction;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoActionCommand;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.StartAutoState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.Automations;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BlueAlliance;
import org.firstinspires.ftc.teamcode.util.Drawing;
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
	private boolean startedActionCommand;
	private AutoActionCommand currentActionCommand;
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
		Pose startPose = startAutoState.getStartPose();
		if (alliance instanceof BlueAlliance) {
			startPose = startPose.mirror();
		}
		this.follower.setStartingPose(startPose);
		this.automationHandler = new Automations(hardwareMap, alliance, true);
		this.autoActions = autoActions;
		this.stateFinished = false;

		this.pathList = new ArrayList<>();
		Pose previousPose = startPose;
		for (AutoAction autoAction : autoActions) {
			PathChain newPathChain = autoAction.getPathChain(follower, previousPose);
			this.pathList.add(newPathChain);
			if (newPathChain != null) {
				previousPose = newPathChain.endPose();
			}
		}
	}

	public StartAutoState[] getStartingStates() {
		return new StartAutoState[] {
				components.new StartLowState(),
				components.new StartUpState(),
		};
	}

	public void start() {
		automationHandler.start();
	}

	public void update() {
		if (currentState < 0)
			return;
		updatePathFollowing();
		updateCommands();

		if (stateFinished) {
			if (currentState + 1 < autoActions.size()) {
				setState(currentState + 1);
			} else {
				setState(-1);
			}
		}
	}

	public void updateCommands() {
		automationHandler.updatePose(follower.getPose());
		automationHandler.updateVelocity(follower.getVelocity());
		automationHandler.updateTurret();
		automationHandler.updateShooter();
		automationHandler.automationLoop();

		if (stateFinished)
			return;
		if (!startedActionCommand) {
			currentActionCommand = autoActions.get(currentState).getActionCommand();
			startedActionCommand = true;
		}
		if (startedActionCommand && currentActionCommand != null) {
			stateFinished = currentActionCommand.run(follower, automationHandler);
		}
	}

	public void updatePathFollowing() {
		follower.update();
		if (!startedFollowingPath) {
			if (currentState < pathList.size()) {
				PathChain currentPath = pathList.get(currentState);
				if (currentPath != null) {
					follower.followPath(currentPath, true);
				}
			}
			startedFollowingPath = true;
		}
	}

	public void end() {
		automationHandler.end();
	}

	private void setState(int state) {
		currentState = state;
		startedFollowingPath = false;
		startedActionCommand = false;
		stateFinished = false;
	}

	public List<AutoAction> getAutoActions() {
		return new ArrayList<>(autoActions);
	}

	public void showTelemetry(TelemetryManager telemetryManager) {
		telemetryManager.addData("State", currentState);
		telemetryManager.addData("Current action", autoActions.get(currentState).getNameString());
		telemetryManager.addData("Storage", automationHandler.getArtifactsStored());
		telemetryManager.addData("Storage State", automationHandler.getStorageState());
		telemetryManager.addData("Storage Intake State", automationHandler.getIntakeState());
		telemetryManager.addData("Storage Transfer State", automationHandler.getTransferState());

		Drawing.drawRobot(follower.getPose(), telemetryManager.getDashboardCanvas());
		Drawing.sendPacket();
	}

	public void showAutomationsTelemetry(TelemetryManager telemetryManager) {
		automationHandler.showTelemetry(telemetryManager);
	}

	public void showFollowerTelemetry(TelemetryManager telemetryManager) {
		telemetryManager.addData("Follower busy", follower.isBusy());
		telemetryManager.addData("Following", pathList.get(currentState));
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
