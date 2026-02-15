package org.firstinspires.ftc.teamcode.util.FateWeaver;

import java.lang.reflect.Method;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;

import com.qualcomm.robotcore.util.WebHandlerManager;

import android.content.Context;
import android.util.Log;
import fi.iki.elonen.NanoHTTPD;
import gay.zharel.fateweaver.flight.LogFiles;

public class WebHandler {
	@WebHandlerRegistrar
	public static void registerWebHandlers(Context context, WebHandlerManager manager) {
		try {
			Method handler1 = LogFiles.class.getDeclaredMethod("registerRoutes$lambda$0", new Class[] {NanoHTTPD.IHTTPSession.class});
			handler1.setAccessible(true);
			manager.register("/fate/logs", (session) -> {
				try {
					return (NanoHTTPD.Response) handler1.invoke(null, session);
				} catch (Exception e) {
					Log.e("FateWeaverWebHandler", "Failed to invoke handler", e);
					return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Server Error");
				}
			});

			Method handler2 = LogFiles.class.getDeclaredMethod("registerRoutes$lambda$1", new Class[] {NanoHTTPD.IHTTPSession.class});
			handler2.setAccessible(true);
			manager.register("/fate/fate/logs/download", (session) -> {
				try {
					return (NanoHTTPD.Response) handler2.invoke(null, session);
				} catch (Exception e) {
					Log.e("FateWeaverWebHandler", "Failed to invoke handler", e);
					return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Server Error");
				}
			});
		} catch (Exception e) {
			Log.e("FateWeaverWebHandler", "Failed to register handlers", e);
		}
	}
}
