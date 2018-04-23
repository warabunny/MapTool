/*
 * This software Copyright by the RPTools.net development team, and licensed under the Affero GPL Version 3 or, at your option, any later version.
 *
 * MapTool Source Code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU Affero General Public License * along with this source Code. If not, please visit <http://www.gnu.org/licenses/> and specifically the Affero license text
 * at <http://www.gnu.org/licenses/agpl.html>.
 */
package net.rptools.maptool.client.ui.zone;

import java.awt.Graphics2D;
import java.util.List;

import javax.swing.SwingWorker;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import net.rptools.maptool.client.MapTool;
import net.rptools.maptool.client.ui.zone.ZoneRenderer.SelectionSet;
import net.rptools.maptool.client.walker.ZoneWalker;
import net.rptools.maptool.model.CellPoint;
import net.rptools.maptool.model.Path;
import net.rptools.maptool.model.TokenFootprint;

public class RenderPathWorker extends SwingWorker<Void, Void> {
	private static final Logger log = LogManager.getLogger(RenderPathWorker.class);

	ZoneRenderer zoneRenderer;
	Graphics2D g;
	ZoneWalker walker;
	TokenFootprint footprint;
	public static boolean isWorking = false;
	public static boolean cancel = false;
	Path<CellPoint> path;
	CellPoint startPoint, endPoint;
	boolean setWaypoint = false;
	private boolean restrictMovement = false;
	int doSomeWork = 0;

	long start = System.currentTimeMillis();

	// RenderPathWorker(ZoneRenderer zoneRenderer, Graphics2D g, ZoneWalker walker, TokenFootprint footprint) {
	// this.zoneRenderer = zoneRenderer;
	// this.g = g;
	// this.walker = walker;
	// this.footprint = footprint;
	// }

	RenderPathWorker(ZoneWalker walker, CellPoint startPoint, CellPoint endPoint) {
		this.walker = walker;
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		doSomeWork = 1;
	}

	public RenderPathWorker(ZoneWalker walker, CellPoint endPoint, boolean restrictMovement) {
		this.walker = walker;
		this.endPoint = endPoint;
		doSomeWork = 2;
		this.restrictMovement = restrictMovement;
	}

	public RenderPathWorker(ZoneWalker walker, CellPoint endPoint, boolean restrictMovement, ZoneRenderer zoneRenderer) {
		this.walker = walker;
		this.endPoint = endPoint;
		doSomeWork = 2;
		this.restrictMovement = restrictMovement;
		this.zoneRenderer = zoneRenderer;
	}

	@Override
	protected Void doInBackground() throws Exception {
		//log.info("start RenderPathWorker, doSomeWork: " + doSomeWork);
		// log.info("Worker working!");
		//isWorking = true;
		// RenderPathWorker something = this;
		// while (!isCancelled()) {
		// Thread.sleep(20);
		// if ((System.currentTimeMillis() - start) > 500)
		// break;
		// }
		//log.info("was cancelled: " + isCancelled());
		// log.info("something? " + (something == null));

		if (doSomeWork == 1) {
			walker.setWaypoints(startPoint, endPoint);
		} else if (doSomeWork == 2) {
			walker.replaceLastWaypoint(endPoint, restrictMovement);
		} else {
			path = walker.getPath(this); // walker.getPath() is where the real magic happens!
			//log.info("Time for walker.getPath(): " + (System.currentTimeMillis() - start) + "ms");
			process(null);
		}
		// process(null); // this could be incremental?
		// ZoneRenderer.setRenderedPath(walker.getPath(something));
		// this.cancel(false);
		// zoneRenderer.renderPath(g, path, footprint);
		// log.info("renderPathWorker.isCancelled? " + isCancelled());

		// while (!this.isCancelled()) {
		// //log.info("Swingworker waiting...");
		// }

		return null;
	}

	@Override
	protected void process(List<Void> v) {
		// if(zoneRenderer != null)
		// zoneRenderer.repaint();

		// zoneRenderer.renderPath(g, path, footprint); // invokes UI thread?
		log.info("process renderPath done: " + (System.currentTimeMillis() - start) + "ms");
	}

	// This is invoked in the UI thread!
	@Override
	protected void done() {
		isWorking = false;
		cancel = false;
		// log.info("Swingworker done! calling renderPath: " + (System.currentTimeMillis() - start) + "ms");
		// zoneRenderer.renderPath(g, path, footprint);
		// ZoneRenderer.setRenderedPath(path);

		if (zoneRenderer != null)
			MapTool.getFrame().getCurrentZoneRenderer().repaint();
		else
			log.info("\n\n\nWhy is zoneRenderer null in RenderPathWorker? SHOULD NEVER SEE THIS!");

		// isStarted = false;
		//log.info("Time for RenderPathWorker: " + (System.currentTimeMillis() - start) + "ms");
	}

}