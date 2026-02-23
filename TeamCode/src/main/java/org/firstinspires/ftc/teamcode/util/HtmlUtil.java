package org.firstinspires.ftc.teamcode.util;

public class HtmlUtil {
	public static String colourText(String text, String color) {
		return "<font color=\"" + color + "\">" + text + "</font>";
	}

	public static String monospaceText(String text) {
		return "<tt>" + text + "</tt>";
	}
}
