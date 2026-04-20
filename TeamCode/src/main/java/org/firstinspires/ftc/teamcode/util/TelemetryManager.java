package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.psilynx.psikit.core.Logger;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import android.text.Html;

/*
 * Class to manage the use of telemetry for FTC, Dashboard, and Panels
 */
public class TelemetryManager {
	private Telemetry ftcTelemetry;
	private FtcDashboard dashboardInstance;
	private com.bylazar.telemetry.TelemetryManager panelsTelemetry;

	private TelemetryPacket dashboardPacket;

	private boolean loggingEnabled = false;
	private boolean htmlMode = false;

	public void setFtcTelemetry(Telemetry ftcTelemetry) {
		this.ftcTelemetry = ftcTelemetry;
		if (htmlMode) {
			this.ftcTelemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
		} else {
			this.ftcTelemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
		}
	}

	public void setFtcFastTelemetry(OpMode opmode) {
		this.ftcTelemetry = new FastTelemetry(opmode);
		if (htmlMode) {
			this.ftcTelemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
		} else {
			this.ftcTelemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
		}
	}

	public void setDashboardInstance (FtcDashboard dashboardInstance) {
		this.dashboardInstance = dashboardInstance;
		this.dashboardPacket = new TelemetryPacket();
	}

	public void setPanelsTelemetry (com.bylazar.telemetry.TelemetryManager panelsTelemetry) {
		this.panelsTelemetry = panelsTelemetry;
	}
	
	public void setLoggingEnabled(boolean logsEnabled) {
		this.loggingEnabled = logsEnabled;
	}

	public void setHtmlMode(boolean htmlMode) {
		if (ftcTelemetry != null) {
			if (htmlMode) {
				this.ftcTelemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
			} else {
				this.ftcTelemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
			}
		}
		this.htmlMode = htmlMode;
	}

	/*
	 * Main methods
	 */

	public void addData(String caption, String format, Object... args) {
		addData(caption, String.format(format, args));
	}

	public void addData(String caption, Object value) {
		String htmlString = caption;
		// if (htmlMode) {
		// 	caption = Html.fromHtml(caption, Html.FROM_HTML_MODE_COMPACT).toString();
		// }
		if (value == null) {
			value = "null";
		}
		if (ftcTelemetry != null) {
			ftcTelemetry.addData(htmlString, value);
		}
		if (dashboardInstance != null) {
			dashboardPacket.put(htmlString, value);
		}
		if (panelsTelemetry != null) {
			panelsTelemetry.addData(caption, value);
		}
		if (loggingEnabled) {
			Logger.recordOutput("Telemetry/" + StringUtil.toPascalCase(caption), value.toString());
		}
	}

	public void addLine() {
		addLine("");
	}

	public void addLine(String lineCaption) {
		String htmlString = lineCaption;
		// if (htmlMode) {
		// 	lineCaption = Html.fromHtml(lineCaption, Html.FROM_HTML_MODE_COMPACT).toString();
		// }
		if (ftcTelemetry != null) {
			ftcTelemetry.addLine(htmlString);
		}
		if (dashboardInstance != null) {
			dashboardPacket.addLine(htmlString);
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
