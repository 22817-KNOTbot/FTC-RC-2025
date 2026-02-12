package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftccommon.external.OnCreate;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.squareup.moshi.Moshi;
import com.squareup.moshi.JsonAdapter;
import com.squareup.moshi.JsonDataException;
import com.squareup.moshi.Types;

import dalvik.system.DexFile;

import android.content.Context;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class ValuesStorage {
	private static final String TAG = "ValuesStorage";
	private static final File VALUES_FILE = new File(AppUtil.ROOT_FOLDER, "ValuesStorage/values.json");
	private static Map<String, Map<String, Object>> valuesMap;
	private static Map<Class, String> classMap = new HashMap<>();

	private static final List<String> IGNORED_PACKAGES = new ArrayList<String>(Arrays.asList(
			// List copied from RC ClassManager.java
			"android", // Also excludes androidx
			"com.android",
			"com.google",
			"com.qualcomm.robotcore.wifi",
			"com.sun",
			"gnu.kawa.swingviews",
			"io.netty",
			"java", // Also excludes javax
			"kawa",
			"org.apache",
			"org.checkerframework",
			"org.firstinspires.ftc.robotcore.internal.android",
			"org.java_websocket",
			"org.slf4j",
			"org.threeten",
			"com.journeyapps"));

	@Retention(RetentionPolicy.RUNTIME)
	@Target(ElementType.TYPE)
	public static @interface StoreValues {
		String value() default "";
	}

	// @TeleOp(name = "Save Values")
	public static class SaveValuesOpMode extends LinearOpMode {
		@Override
		public void runOpMode() {
			waitForStart();
			saveValues();
			requestOpModeStop();
		}
	}

	@SuppressWarnings("unchecked")
	// @OnCreate
	public static void initialize(Context context) {
		Log.i(TAG, "Initializing");
		valuesMap = parseJson(readFile(VALUES_FILE));
		ValuesStorage.class.getClassLoader();

		List<String> allClassNames;
		try {
			allClassNames = new ArrayList<String>(
					Collections.list(new DexFile(context.getPackageCodePath()).entries()));
		} catch (IOException err) {
			Log.e(TAG, "Error getting DexFile entries", err);
			return;
		}

		for (String className : allClassNames) {
			boolean ignore = false;
			for (String packageName : IGNORED_PACKAGES) {
				if (className.startsWith(packageName)) {
					ignore = true;
					break;
				}
			}
			if (ignore)
				continue;

			try {
				Class clazz = Class.forName(className, false, ValuesStorage.class.getClassLoader());

				if (!clazz.isAnnotationPresent(StoreValues.class))
					continue;

				String providedName = ((StoreValues) clazz.getAnnotation(StoreValues.class)).value();
				String name = !providedName.isEmpty() ? providedName : clazz.getSimpleName();

				classMap.put(clazz, name);
			} catch (ClassNotFoundException | NoClassDefFoundError err) {
				continue;
			}
		}

		updateValues();
	}

	public static void updateValues() {
		Log.i(TAG, "Started updating values");
		classMap.forEach((clazz, name) -> {
			Map<String, Object> storedValues = valuesMap.get(name);
			if (storedValues == null)
				return;

			storedValues.forEach((propertyName, value) -> {
				Field field = null;
				try {
					field = clazz.getField(propertyName);
				} catch (NoSuchFieldException err) {
					return;
				}

				int modifier = field.getModifiers();
				if (Modifier.isPublic(modifier) && Modifier.isStatic(modifier) && !Modifier.isFinal(modifier)) {
					try {
						// Convert from double to float if necessary
						if (value instanceof Number) {
							if (field.get(null).getClass().equals(Double.class)) {
								value = ((Number) value).doubleValue();
							} else if (field.get(null).getClass().equals(Float.class)) {
								value = ((Number) value).floatValue();
							} else if (field.get(null).getClass().equals(Integer.class)) {
								value = ((Number) value).intValue();
							}
						}

						field.set(null, field.get(null).getClass().cast(value));
					} catch (ClassCastException | IllegalArgumentException err) {
						Log.w(TAG, "Invalid value stored: type does not match. Given value: \"" + value + "\" of type "
								+ value.getClass() + ". Expected type " + field.getType());
					} catch (IllegalAccessException err) {
						Log.e(TAG, "Illegal access exception", err);
					}
				}
			});

			Log.d(TAG, "Updated values for " + name);
		});
	}

	public static void saveValues() {
		Log.i(TAG, "Started saving values");
		classMap.forEach((clazz, name) -> {
			Map<String, Object> values = new HashMap<>();
			for (Field field : clazz.getFields()) {
				int modifier = field.getModifiers();
				if (!Modifier.isPublic(modifier) || !Modifier.isStatic(modifier) || Modifier.isFinal(modifier))
					continue;

				String fieldName = field.getName();
				Object value = null;
				try {
					value = field.get(null);
				} catch (IllegalAccessException err) {
					Log.e(TAG, "IllegalAccessException while getting " + fieldName + " in " + name, err);
				}

				if (value != null)
					values.put(fieldName, value);
			}

			valuesMap.put(name, values);
		});

		writeFile(VALUES_FILE, buildJson(valuesMap));
		Log.i(TAG, "Finished saving values");
	}

	private static String readFile(File file) {
		if (!file.exists() || !file.isFile()) {
			try {
				file.getParentFile().mkdirs();
				file.createNewFile();
			} catch (IOException err) {
				Log.e(TAG, "Error initializing values file", err);
			}
		}
		if (!file.canRead())
			return "";

		String text = "";

		try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
			String line = reader.readLine();
			while (line != null) {
				text += line;
				line = reader.readLine();
			}
		} catch (IOException err) {
			Log.e(TAG, "Error reading file \"" + file.getAbsolutePath() + "\"", err);
		}

		return text;
	}

	private static void writeFile(File file, String text) {
		if (!file.exists() || !file.isFile()) {
			try {
				file.getParentFile().mkdirs();
				file.createNewFile();
			} catch (IOException err) {
				Log.e(TAG, "Error initializing values file", err);
			}
		}

		try (FileOutputStream stream = new FileOutputStream(file)) {
			stream.write(text.getBytes());
		} catch (IOException err) {
			Log.e(TAG, "Error writing to file \"" + file.getAbsolutePath() + "\"", err);
		}
	}

	private static Map<String, Map<String, Object>> parseJson(String json) {
		Moshi moshi = new Moshi.Builder().build();
		JsonAdapter<Map<String, Map<String, Object>>> jsonAdapter = moshi
				.adapter(Types.newParameterizedType(Map.class, String.class, Map.class));

		if (!json.startsWith("{")) {
			Log.i(TAG, "Initialized values map");
			return new HashMap<String, Map<String, Object>>();
		}

		try {
			Map data = jsonAdapter.fromJson(json);
			return data;
		} catch (IOException | JsonDataException err) {
			Log.e(TAG, "Error converting JSON data given json: \"" + json + "\"", err);
		}

		return null;
	}

	private static String buildJson(Map<String, Map<String, Object>> data) {
		Moshi moshi = new Moshi.Builder().build();
		JsonAdapter<Map<String, Map<String, Object>>> jsonAdapter = moshi
				.adapter(Types.newParameterizedType(Map.class, String.class, Map.class));

		String json = jsonAdapter.toJson(data);
		return json;
	}
}
