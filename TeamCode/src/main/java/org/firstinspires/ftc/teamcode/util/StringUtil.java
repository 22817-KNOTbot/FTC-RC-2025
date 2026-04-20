package org.firstinspires.ftc.teamcode.util;

public class StringUtil {
	public static String toPascalCase(String s) {
		String[] parts = s.split(" ");
		String pascalCaseString = "";
		for (String part : parts) {
			pascalCaseString = pascalCaseString + toProperCase(part);
		}
		return pascalCaseString;
	}

	public static String toProperCase(String s) {
		return s.substring(0, 1).toUpperCase() +
				   s.substring(1).toLowerCase();
	}
}
