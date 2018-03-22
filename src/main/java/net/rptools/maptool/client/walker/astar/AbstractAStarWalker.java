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

import net.rptools.maptool.client.walker.AbstractZoneWalker;
import net.rptools.maptool.model.CellPoint;
import net.rptools.maptool.model.Label;
import net.rptools.maptool.model.Zone;

public abstract class AbstractAStarWalker extends AbstractZoneWalker {
	private static final Logger log = LogManager.getLogger(AbstractAStarWalker.class);

	public AbstractAStarWalker(Zone zone) {
		super(zone);
	}

	private int distance = -1;

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

		openList.add(new AStarCellPoint(start));
		openSet.put(openList.get(0), openList.get(0));

		AStarCellPoint node = null;
		Area vbl = zone.getTopology();

		// If end point is in VBL GTFO
		//if (vbl.intersects(zone.getGrid().getBounds(end)))
			//return null;

		long timeOut = System.currentTimeMillis() + 2000;
		// log.info("openList size: " + openList.size());

		// erase all labels
		List<Label> allLabels = zone.getLabels();
		for (Label label : allLabels) {
			zone.removeLabel(label.getId());
		}

		while (!openList.isEmpty()) {
			if (System.currentTimeMillis() > timeOut) {
				log.info("Timing out...");
				log.info("openList size now: " + openList.size());
				break;
			}

			node = openList.remove(0);
			openSet.remove(node);
			if (node.equals(end)) {
				break;
			}
			// int[][] allowedMap = getNeighborMap(node.x, node.y);
			int[][] neighborMap = getNeighborMap(node.x, node.y);
			// List<int[]> neighborMap = new ArrayList<int[]>();

			// Remove any cells that intersect with VBL
			// for (int i = 0; i < allowedMap.length; i++) {
			// int x = node.x + allowedMap[i][0];
			// int y = node.y + allowedMap[i][1];
			// AStarCellPoint allowedNode = new AStarCellPoint(x, y);
			//
			// Rectangle cellBounds = zone.getGrid().getBounds(allowedNode);
			// boolean blocksMovement = vbl.intersects(cellBounds);
			//
			// // System.out.println("Area bonds: " + vbl.getBounds()); // This is in map coords/grid pixels
			// System.out.println("x,y:" + x + "," + y + " :: block? " + blocksMovement); // This is in cells, durr
			// // System.out.println("cell bounds? " + zone.getGrid().getBounds(neighborNode));
			//
			// //if (!blocksMovement)
			// neighborMap.add(new int[] { x, y });
			// }

			// System.out.println("Final list size is: " + neighborMap.size());

			for (int i = 0; i < neighborMap.length; i++) {
				int x = node.x + neighborMap[i][0];
				int y = node.y + neighborMap[i][1];
				AStarCellPoint neighborNode = new AStarCellPoint(x, y);
				if (closedSet.contains(neighborNode)) {
					continue;
				}

				// log.info("isSecondDiag: " + isSecondDiag(node));

				neighborNode.parent = node;
				neighborNode.gScore = gScore(node, neighborNode) * 10;
				neighborNode.hScore = hScore(neighborNode, end) * 10;

				Rectangle cellBounds = zone.getGrid().getBounds(neighborNode);
				double perc = 0;
				
				if (vbl.intersects(cellBounds)) {
					Area cellArea = new Area(cellBounds);
					cellArea.intersect(vbl);
					Rectangle overlap = cellArea.getBounds();
					perc = 1 + (area(overlap) / area(cellBounds)); // * 100;

					neighborNode.gScore = neighborNode.gScore * perc;
					neighborNode.hScore = neighborNode.hScore * perc;

					// log.debug("overlap % = " + perc);
					// if (perc >= 50) {
					// neighborNode.gScore = neighborNode.gScore + perc * 1;
					// neighborNode.hScore = neighborNode.hScore + perc * 1;
					// log.debug("walking around vbl...");
					// } else {
					// neighborNode.gScore = neighborNode.gScore + perc;
					// neighborNode.hScore = neighborNode.hScore + perc;
					// }
				}

				// if(isSecondDiag(node)) {
				// Label diag = new Label();
				// diag.setLabel("1.5");
				// diag.setX(cellBounds.x + 45);
				// diag.setY(cellBounds.y + 45);
				// diag.setForegroundColor(Color.RED);
				// zone.putLabel(diag);
				// }

				Label gScore = new Label();
				Label hScore = new Label();
				Label fScore = new Label();

				gScore.setLabel("" + Math.round(neighborNode.gScore));
				gScore.setX(cellBounds.x + 10);
				gScore.setY(cellBounds.y + 10);

				hScore.setLabel("" + Math.round(neighborNode.hScore));
				hScore.setX(cellBounds.x + 35);
				hScore.setY(cellBounds.y + 10);

				fScore.setLabel("" + Math.round(neighborNode.gScore + neighborNode.hScore));
				fScore.setX(cellBounds.x + 25);
				fScore.setY(cellBounds.y + 35);
				fScore.setForegroundColor(Color.RED);

				// zone.putLabel(gScore);
				// zone.putLabel(hScore);
				hScore.setLabel("" + perc);
				zone.putLabel(hScore);
				zone.putLabel(fScore);

				if (openSet.containsKey(neighborNode)) {
					AStarCellPoint oldNode = openSet.get(neighborNode);
					// check if it is cheaper to get here the way that we just
					// came, versus the previous path
					if (neighborNode.gScore < oldNode.gScore) {
						oldNode.gScore = neighborNode.gScore;
						neighborNode = oldNode;
						neighborNode.parent = node;
					}
					continue;
				}
				pushNode(openList, neighborNode);
				openSet.put(neighborNode, neighborNode);
			}
			closedSet.add(node);
			node = null;
		}
		List<CellPoint> ret = new LinkedList<CellPoint>();
		while (node != null) {
			ret.add(node);
			node = node.parent;
		}
		distance = -1;
		Collections.reverse(ret);
		// log.info(ret);

		return ret;
	}

	private double area(Rectangle r) {
		return r.getWidth() * r.getHeight();
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
}
