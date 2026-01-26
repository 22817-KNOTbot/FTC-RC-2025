package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
 * Class to manage the use of telemetry for FTC, Dashboard, and Panels
 */
public class TelemetryManager {
	private Telemetry ftcTelemetry;
	private FtcDashboard dashboardInstance;
	private com.bylazar.telemetry.TelemetryManager panelsTelemetry;

	private TelemetryPacket dashboardPacket;

	public void setFtcTelemetry(Telemetry ftcTelemetry) {
		this.ftcTelemetry = ftcTelemetry;
	}

	public void setFtcFastTelemetry(OpMode opmode) {
		this.ftcTelemetry = new FastTelemetry(opmode);
	}

	public void setDashboardInstance (FtcDashboard dashboardInstance) {
		this.dashboardInstance = dashboardInstance;
		this.dashboardPacket = new TelemetryPacket();
	}

	public void setPanelsTelemetry (com.bylazar.telemetry.TelemetryManager panelsTelemetry) {
		this.panelsTelemetry = panelsTelemetry;
	}

	/*
	 * Main methods
	 */

	public void addData(String caption, String format, Object... args) {
		addData(caption, String.format(format, args));
	}

	public void addData(String caption, Object value) {
		if (value == null) {
			value = "null";
		}
		if (ftcTelemetry != null) {
			ftcTelemetry.addData(caption, value);
		}
		if (dashboardInstance != null) {
			dashboardPacket.put(caption, value);
		}
		if (panelsTelemetry != null) {
			panelsTelemetry.addData(caption, value);
		}
	}

	public void addLine() {
		addLine("");
	}

	public void addLine(String lineCaption) {
		if (ftcTelemetry != null) {
			ftcTelemetry.addLine(lineCaption);
		}
		if (dashboardInstance != null) {
			dashboardPacket.addLine(lineCaption);
		}
		if (panelsTelemetry != null) {
			panelsTelemetry.addLine(lineCaption);
		}
	}

	public void update() {
		if (ftcTelemetry != null) {
			ftcTelemetry.update();
		}
		if (dashboardInstance != null) {
			dashboardInstance.sendTelemetryPacket(dashboardPacket);
			dashboardPacket = new TelemetryPacket();
		}
		if (panelsTelemetry != null) {
			panelsTelemetry.update();
		}
	}

	/*
	 * Dashboard drawing
	 */
	public Canvas getDashboardCanvas() {
		return dashboardPacket.fieldOverlay();
	}
}
