/*
 * This software Copyright by the RPTools.net development team, and licensed under the Affero GPL Version 3 or, at your option, any later version.
 *
 * MapTool Source Code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU Affero General Public License * along with this source Code. If not, please visit <http://www.gnu.org/licenses/> and specifically the Affero license text
 * at <http://www.gnu.org/licenses/agpl.html>.
 */
package net.rptools.maptool.client.walker.astar;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.geom.Area;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.locationtech.jts.awt.ShapeReader;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;

import net.rptools.maptool.client.MapTool;
import net.rptools.maptool.client.walker.AbstractZoneWalker;
import net.rptools.maptool.model.CellPoint;
import net.rptools.maptool.model.GUID;
import net.rptools.maptool.model.Label;
import net.rptools.maptool.model.TokenFootprint;
import net.rptools.maptool.model.Zone;

public abstract class AbstractAStarWalker extends AbstractZoneWalker {
	private static final Logger log = LogManager.getLogger(AbstractAStarWalker.class);

	public AbstractAStarWalker(Zone zone) {
		super(zone);
	}

	protected Area vbl = new Area();
	protected double normal_cost = 1; // zone.getUnitsPerCell();

	double distance = -1;
	private List<GUID> debugLabels;
	private boolean debugCosts = false; // Manually set this to view H, G & F costs as rendered labels

	protected final GeometryFactory geometryFactory = new GeometryFactory();
	protected ShapeReader shapeReader = new ShapeReader(geometryFactory);
	protected Geometry vblGeometry = null;
	protected TokenFootprint footprint;

	protected Map<AStarCellPoint, AStarCellPoint> checkedList = new ConcurrentHashMap<AStarCellPoint, AStarCellPoint>();
	protected long avgRetrieveTime;
	protected long avgTestTime;
	protected long retrievalCount;
	protected long testCount;

	/**
	 * Returns the list of neighbor cells that are valid for being movement-checked. This is an array of (x,y) offsets (see the constants in this class) named as compass points.
	 * <p>
	 * It should be possible to query the current (x,y) CellPoint passed in to determine which directions are feasible to move into. But it would require information about visibility (which token is
	 * moving, does it have sight, and so on). Currently that information is not available here, but perhaps an option Token parameter could be specified to the constructor? Or maybe as the tree was
	 * scanned, since I believe all Grids share a common ZoneWalker.
	 */
	protected abstract int[][] getNeighborMap(int x, int y);

	@Override
	public void setFootprint(TokenFootprint footprint) {
		this.footprint = footprint;
	}

	@Override
	protected List<CellPoint> calculatePath(CellPoint start, CellPoint end) {
		List<AStarCellPoint> openList = new ArrayList<AStarCellPoint>();
		Map<AStarCellPoint, AStarCellPoint> openSet = new HashMap<AStarCellPoint, AStarCellPoint>(); // For faster lookups
		Set<AStarCellPoint> closedSet = new HashSet<AStarCellPoint>();

		// Current fail safe... bail out after 1/2 second of searching
		long timeOut = System.currentTimeMillis();

		if (debugLabels == null)
			debugLabels = new ArrayList<GUID>();

		// if (start.equals(end))
		// log.info("NO WORK!");

		openList.add(new AStarCellPoint(start));
		openSet.put(openList.get(0), openList.get(0));

		AStarCellPoint currentNode = null;

		// Get current VBL for map...
		// FIXME: Reduce VBL to bounds...?? sub millisecond performance, don't need!?
		// Using JTS because AWT Area can only intersect with Area and we want to use simple lines here.
		// Render VBL to Geometry class once and store. FIXME: Add to Zone and refresh on Topology...
		vbl = MapTool.getFrame().getCurrentZoneRenderer().getZoneView().getTopologyTree().getArea();
		if (!vbl.isEmpty()) {
			try {
				vblGeometry = shapeReader.read(vbl.getPathIterator(null)).buffer(1); // .buffer helps creating valid geometry and prevent self-intersecting polygons
				if (!vblGeometry.isValid())
					log.info("vblGeometry is invalid! May cause issues. Check for self-intersecting polygons.");
			} catch (Exception e) {
				log.info("vblGeometry oh oh: ", e);
			}

			// log.info("vblGeometry bounds: " + vblGeometry.toString());
		}

		// Erase previous debug labels, this actually erases ALL labels!
		if (!zone.getLabels().isEmpty() && debugCosts) {
			for (Label label : zone.getLabels()) {
				zone.removeLabel(label.getId());
			}
		}

		// If debug is enabled, MapTool is pretty busy so...
		double estimatedTimeoutNeeded = 10000;
		// if (log.isDebugEnabled()) {
		// estimatedTimeoutNeeded = hScore(start, end) * 20;
		// } else {
		// estimatedTimeoutNeeded = Math.max(hScore(start, end) / 5, 6000);
		// }

		// Timeout quicker for GM cause reasons
		if (MapTool.getPlayer().isGM())
			estimatedTimeoutNeeded = estimatedTimeoutNeeded / 2;

		// log.info("A* Path timeout estimate: " + estimatedTimeoutNeeded);

		while (!openList.isEmpty()) {
			if (System.currentTimeMillis() > timeOut + estimatedTimeoutNeeded) {
				log.info("Timing out after " + estimatedTimeoutNeeded);
				break;
			}

			currentNode = openList.remove(0);
			openSet.remove(currentNode);
			if (currentNode.equals(end)) {
				break;
			}

			// if(vbl.contains(currentNode.toPoint()))
			// break;

			// if(vbl.contains(start.x, start.y))
			// return null;

			for (AStarCellPoint neighborNode : getNeighbors(currentNode, closedSet)) {
				neighborNode.h = hScore(neighborNode, end);
				showDebugInfo(neighborNode);

				if (openSet.containsKey(neighborNode)) {
					// check if it is cheaper to get here the way that we just came, versus the previous path
					AStarCellPoint oldNode = openSet.get(neighborNode);
					if (neighborNode.getG() < oldNode.getG()) {
						oldNode.replaceG(neighborNode);
						neighborNode = oldNode;
						neighborNode.parent = currentNode;
					}
					continue;
				}

				pushNode(openList, neighborNode);
				openSet.put(neighborNode, neighborNode);
			}

			closedSet.add(currentNode);
			currentNode = null;

			// down stream SwingWorker. Need to cancel here if called

			if (Thread.interrupted()) {
				// log.info("Thread interrupted!");
				openList.clear();
			}
		}

		List<CellPoint> ret = new LinkedList<CellPoint>();
		while (currentNode != null) {
			ret.add(currentNode);
			currentNode = currentNode.parent;
		}

		// distance = -1;
		// Jamz We don't need to "calculate" distance after the fact as it's already stored as the G cost...
		// mmm ok have to deal with this and waypoints reset this...
		if (!ret.isEmpty())
			distance = ret.get(0).getDistanceTraveled(zone);
		else
			distance = 0;

		Collections.reverse(ret);
		timeOut = (System.currentTimeMillis() - timeOut);
		if (timeOut > 50)
			log.info("Time to calculate A* path warning: " + timeOut + "ms");

		// if (retrievalCount > 0)
		// log.info("avgRetrieveTime: " + Math.floor(avgRetrieveTime / retrievalCount)/1000 + " micro");
		// if (testCount > 0)
		// log.info("avgTestTime: " + Math.floor(avgTestTime / testCount)/1000 + " micro");

		return ret;
	}

	void pushNode(List<AStarCellPoint> list, AStarCellPoint node) {
		if (list.isEmpty()) {
			list.add(node);
			return;
		}
		if (node.cost() < list.get(0).cost()) {
			list.add(0, node);
			return;
		}
		if (node.cost() > list.get(list.size() - 1).cost()) {
			list.add(node);
			return;
		}
		for (ListIterator<AStarCellPoint> iter = list.listIterator(); iter.hasNext();) {
			AStarCellPoint listNode = iter.next();
			if (listNode.cost() > node.cost()) {
				iter.previous();
				iter.add(node);
				return;
			}
		}
	}

	protected abstract List<AStarCellPoint> getNeighbors(AStarCellPoint node, Set<AStarCellPoint> closedSet);

	protected abstract int calculateDistance(List<CellPoint> path, int feetPerCell);

	protected abstract double gScore(CellPoint p1, CellPoint p2);

	protected abstract double hScore(CellPoint p1, CellPoint p2);

	public int getDistance() {
		if (distance == -1) {
			distance = calculateDistance(getPath().getCellPath(), getZone().getUnitsPerCell());
		}

		// log.info("Feet Distance is " + distance * getZone().getUnitsPerCell());

		return (int) distance; // * getZone().getUnitsPerCell();
	}

	protected void showDebugInfo(AStarCellPoint node) {
		if (!log.isDebugEnabled() && !debugCosts)
			return;

		Rectangle cellBounds = zone.getGrid().getBounds(node);
		DecimalFormat f = new DecimalFormat("##.00");

		Label gScore = new Label();
		Label hScore = new Label();
		Label fScore = new Label();

		gScore.setLabel(f.format(node.getG()));
		gScore.setX(cellBounds.x + 10);
		gScore.setY(cellBounds.y + 10);

		hScore.setLabel(f.format(node.h));
		hScore.setX(cellBounds.x + 35);
		hScore.setY(cellBounds.y + 10);

		fScore.setLabel(f.format(node.cost()));
		fScore.setX(cellBounds.x + 25);
		fScore.setY(cellBounds.y + 25);
		fScore.setForegroundColor(Color.RED);

		zone.putLabel(gScore);
		zone.putLabel(hScore);
		zone.putLabel(fScore);

		// Track labels to delete later
		debugLabels.add(gScore.getId());
		debugLabels.add(hScore.getId());
		debugLabels.add(fScore.getId());
	}

	public Collection<AStarCellPoint> getCheckedPoints() {
		return checkedList.values();
	}
}
