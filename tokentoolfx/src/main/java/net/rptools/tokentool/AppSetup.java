/*
 * The MIT License
 * 
 * Copyright (c) 2005 David Rice, Trevor Croft
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package net.rptools.tokentool;

import java.io.File;
import java.io.IOException;

import org.apache.commons.io.FileUtils;

import javafx.scene.control.TreeItem;
import net.rptools.lib.AppUtil;
import net.rptools.lib.FileUtil;
import net.rptools.lib.image.ImageUtil;

/**
 * Executes only the first time the application is run.
 */
public class AppSetup {
	private static final String DEFAULT_TOKEN_ZIP = "net/rptools/tokentool/zip/overlays.zip";

	public static void install(String versionString) {
		AppUtil.init("tokentoolfx");

		File appDir = AppUtil.getAppHome();
		File overlayDir = AppConstants.OVERLAY_DIR;
		File[] overLays = overlayDir.listFiles(ImageUtil.SUPPORTED_IMAGE_FILE_FILTER);
		File overlayVer = new File(appDir.getAbsolutePath() + "/version.txt");

		// Only init once or if version.text is missing
		// After 1.4.0.1 we can install only newer overlays based on version if needed
		try {
			if (overlayVer.exists() && overLays.length > 0) {
				return;
			} else if (!overlayVer.exists()) {
				//overlayVer.createNewFile();
				FileUtils.writeStringToFile(overlayVer, versionString);
			}

			if (overLays.length > 0) {
				File backupDir = new File(overlayDir.getAbsolutePath() + "/Backup");
				System.out.println("Log: Backing up existing overlays to " + backupDir.getAbsolutePath());
				for (File file : overLays)
					FileUtils.moveFileToDirectory(file, backupDir, true);
			}

			// Put in a default samples
			FileUtil.unzip(DEFAULT_TOKEN_ZIP, overlayDir);
		} catch (IOException ioe) {
			ioe.printStackTrace();
		}
	}

	public static void installDefaultOverlays() throws IOException {
		// Create the overlay directory
		File overlayDir = AppConstants.OVERLAY_DIR;
		overlayDir.mkdirs();

		// Put in a default samples
		FileUtil.unzip(DEFAULT_TOKEN_ZIP, overlayDir);
	}
}