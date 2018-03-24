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
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import net.rptools.maptool.client.MapTool;
import net.rptools.maptool.client.ui.zone.ZoneRenderer;
import net.rptools.maptool.client.walker.AbstractZoneWalker;
import net.rptools.maptool.model.CellPoint;
import net.rptools.maptool.model.GUID;
import net.rptools.maptool.model.Label;
import net.rptools.maptool.model.Zone;

public abstract class AbstractAStarWalker extends AbstractZoneWalker {
	private static final Logger log = LogManager.getLogger(AbstractAStarWalker.class);

	public AbstractAStarWalker(Zone zone) {
		super(zone);
	}

	private int distance = -1;
	private List<GUID> debugLabels;

	protected Area vbl;

	/**
	 * Returns the list of neighbor cells that are valid for being movement-checked. This is an array of (x,y) offsets (see the constants in this class) named as compass points.
	 * <p>
	 * It should be possible to query the current (x,y) CellPoint passed in to determine which directions are feasible to move into. But it would require information about visibility (which token is
	 * moving, does it have sight, and so on). Currently that information is not available here, but perhaps an option Token parameter could be specified to the constructor? Or maybe as the tree was
	 * scanned, since I believe all Grids share a common ZoneWalker.
	 */
	protected abstract int[][] getNeighborMap(int x, int y);

	@Override
	protected List<CellPoint> calculatePath(CellPoint start, CellPoint end) {
		List<AStarCellPoint> openList = new ArrayList<AStarCellPoint>();
		Map<AStarCellPoint, AStarCellPoint> openSet = new HashMap<AStarCellPoint, AStarCellPoint>(); // For faster lookups
		Set<AStarCellPoint> closedSet = new HashSet<AStarCellPoint>();

		// Current fail safe... bail out after 1/2 second of searching
		long timeOut = System.currentTimeMillis();

		if (debugLabels == null)
			debugLabels = new ArrayList<GUID>();

		openList.add(new AStarCellPoint(start));
		openSet.put(openList.get(0), openList.get(0));

		AStarCellPoint currentNode = null;
		// Get current VBL for map...
		vbl = MapTool.getFrame().getCurrentZoneRenderer().getZoneView().getTopologyTree().getArea();

		// Erase previous debug labels, this actually erases ALL labels!
		if (!zone.getLabels().isEmpty()) {
			for (Label label : zone.getLabels()) {
				zone.removeLabel(label.getId());
			}
		}

		// If debug is enabled, MapTool is pretty busy so...
		double estimatedTimeoutNeeded = 50;
		if (log.isDebugEnabled()) {
			estimatedTimeoutNeeded = hScore(start, end) * 10;
			log.debug("A* Path timeout estimate: " + estimatedTimeoutNeeded);
		}

		while (!openList.isEmpty()) {
			if (System.currentTimeMillis() > timeOut + estimatedTimeoutNeeded) {
				log.info("Timing out...");
				break;
			}

			currentNode = openList.remove(0);
			openSet.remove(currentNode);
			if (currentNode.equals(end)) {
				break;
			}

			for (AStarCellPoint neighborNode : getNeighbors(currentNode, closedSet)) {
				neighborNode.h = hScore(neighborNode, end);
				showDebugInfo(neighborNode);

				if (openSet.containsKey(neighborNode)) {
					// check if it is cheaper to get here the way that we just came, versus the previous path
					AStarCellPoint oldNode = openSet.get(neighborNode);
					if (neighborNode.g < oldNode.g) {
						oldNode.g = neighborNode.g;
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
		}

		List<CellPoint> ret = new LinkedList<CellPoint>();
		while (currentNode != null) {
			ret.add(currentNode);
			currentNode = currentNode.parent;
		}

		distance = -1;
		Collections.reverse(ret);
		log.info("Time to calculate A* path: " + (System.currentTimeMillis() - timeOut) + "ms");
		return ret;
	}

	private void pushNode(List<AStarCellPoint> list, AStarCellPoint node) {
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
		return distance;
	}

	protected boolean isSecondDiag(AStarCellPoint node) {
		List<CellPoint> path = new LinkedList<CellPoint>();
		while (node != null) {
			path.add(node);
			node = node.parent;
		}

		if (path == null || path.size() == 0)
			return false;

		int numDiag = 0;

		CellPoint previousPoint = null;
		for (CellPoint point : path) {
			if (previousPoint != null) {
				int change = Math.abs(previousPoint.x - point.x) + Math.abs(previousPoint.y - point.y);

				switch (change) {
				case 1:
					break;
				case 2:
					numDiag++;
					break;
				default:
					assert false : String.format("Illegal path, cells are not contiguous; change=%d", change);
					return false;
				}
			}
			previousPoint = point;
		}

		if ((numDiag % 2) == 0)
			return false;
		else
			return true;
	}

	protected void showDebugInfo(AStarCellPoint node) {
		if (!log.isDebugEnabled())
			return;

		Rectangle cellBounds = zone.getGrid().getBounds(node);

		Label gScore = new Label();
		Label hScore = new Label();
		Label fScore = new Label();

		gScore.setLabel("" + Math.round(node.g));
		gScore.setX(cellBounds.x + 10);
		gScore.setY(cellBounds.y + 10);

		hScore.setLabel("" + Math.round(node.h));
		hScore.setX(cellBounds.x + 35);
		hScore.setY(cellBounds.y + 10);

		fScore.setLabel("" + Math.round(node.g + node.h));
		fScore.setX(cellBounds.x + 25);
		fScore.setY(cellBounds.y + 35);
		fScore.setForegroundColor(Color.RED);

		zone.putLabel(gScore);
		zone.putLabel(hScore);
		zone.putLabel(fScore);

		// Track labels to delete later
		debugLabels.add(gScore.getId());
		debugLabels.add(hScore.getId());
		debugLabels.add(fScore.getId());
	}
}
