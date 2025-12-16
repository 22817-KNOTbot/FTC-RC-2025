package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.auto.AutoComponents;
import org.firstinspires.ftc.teamcode.auto.AutoComponents.AutoState;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class AutoManager {
	private AutoComponents components;
	public AutoManager(Alliance alliance) {
		this.components = new AutoComponents(alliance);
	}

	public AutoState[] getStartingStates() {
		return new AutoState[] {
			components.new StartLowState(),
			components.new StartUpState(),
		};
	}
}
