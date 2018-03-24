/*
 * This software Copyright by the RPTools.net development team, and licensed under the Affero GPL Version 3 or, at your option, any later version.
 *
 * MapTool Source Code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU Affero General Public License * along with this source Code. If not, please visit <http://www.gnu.org/licenses/> and specifically the Affero license text
 * at <http://www.gnu.org/licenses/agpl.html>.
 */
package net.rptools.maptool.client.walker.astar;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;

import net.rptools.maptool.client.walker.WalkerMetric;
import net.rptools.maptool.model.CellPoint;
import net.rptools.maptool.model.Zone;

public class AStarSquareEuclideanWalker extends AbstractAStarWalker {
	private static final int[] NORTH = { 0, -1 };
	private static final int[] WEST = { -1, 0 };
	private static final int[] SOUTH = { 0, 1 };
	private static final int[] EAST = { 1, 0 };
	private static final int[] NORTH_EAST = { 1, -1 };
	private static final int[] SOUTH_EAST = { 1, 1 };
	private static final int[] NORTH_WEST = { -1, -1 };
	private static final int[] SOUTH_WEST = { -1, 1 };

	private final WalkerMetric metric;

	private final int[][] neighborMap;
	double normal_cost = 10; // Can vary this later for some reason?
	double diagonal_cost = normal_cost;

	public AStarSquareEuclideanWalker(Zone zone, WalkerMetric metric) {
		super(zone);
		this.metric = metric;

		// If we exposed this list of coordinates to the user, they could define their own movement
		// criteria, including whether to favor the diagonals or the non-diagonals.
		switch (metric) {
		case NO_DIAGONALS:
			neighborMap = new int[][] { NORTH, EAST, SOUTH, WEST };
			break;
		case ONE_TWO_ONE:
			diagonal_cost = normal_cost * 1.5; // 1.4 would be closer math wise but in game terms you count every other square so 1.5
			neighborMap = new int[][] { NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST, NORTH, EAST, SOUTH, WEST };
			break;
		case ONE_ONE_ONE:
		case MANHATTAN:
			// promote straight directions to avoid 'only-diagonals' effect
			neighborMap = new int[][] { NORTH, EAST, SOUTH, WEST, NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST };
			break;
		default:
			// promote diagonals over straight directions by putting them at the front of the array
			neighborMap = new int[][] { NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST, NORTH, EAST, SOUTH, WEST };
			break;
		}
	}

	@Override
	protected List<AStarCellPoint> getNeighbors(AStarCellPoint node, Set<AStarCellPoint> closedSet) {
		List<AStarCellPoint> neighbors = new ArrayList<AStarCellPoint>();
		int[][] neighborMap = getNeighborMap(node.x, node.y);

		// Find all the neighbors.
		for (int[] i : neighborMap) {
			AStarCellPoint neighbor = new AStarCellPoint(node.x + i[0], node.y + i[1]);

			if (closedSet.contains(neighbor))
				continue;

			Rectangle cellBounds = zone.getGrid().getBounds(neighbor);

			// VBL Check
			if (vbl.intersects(cellBounds))
				continue;

			// Add the cell we're coming from
			neighbor.parent = node;

			// Simple check if it's a diagonal neighbor
			if (Arrays.equals(i, NORTH_EAST) || Arrays.equals(i, SOUTH_EAST) || Arrays.equals(i, SOUTH_WEST) || Arrays.equals(i, NORTH_WEST))
				neighbor.g = node.g + diagonal_cost;
			else
				neighbor.g = node.g + normal_cost;

			neighbors.add(neighbor);
			// showDebugInfo(cellBounds, neighbor);
		}

		return neighbors;
	}

	@Override
	public int[][] getNeighborMap(int x, int y) {
		return neighborMap;
	}

	@Override
	protected double gScore(CellPoint p1, CellPoint p2) {
		return metricDistance(p1, p2);
	}

	@Override
	protected double hScore(CellPoint p1, CellPoint p2) {
		return metricDistance(p1, p2);
	}

	private double metricDistance(CellPoint node, CellPoint goal) {
		int xDist = node.x - goal.x;
		int yDist = node.y - goal.y;

		final double distance;

		switch (metric) {
		case MANHATTAN:
		case NO_DIAGONALS:
			distance = Math.abs(xDist) + Math.abs(yDist);
			break;
		default:
		case ONE_TWO_ONE:
			// distance = Math.sqrt(xDist * xDist + yDist * yDist);
			xDist = Math.abs(node.x - goal.x);
			yDist = Math.abs(node.y - goal.y);
			if (xDist > yDist)
				distance = 14 * yDist + 10 * (xDist - yDist);
			else
				distance = 14 * xDist + 10 * (yDist - xDist);
			break;
		case ONE_ONE_ONE:
			distance = Math.max(Math.abs(xDist), Math.abs(yDist));
			break;
		}

		return distance;
	}

	@Override
	protected int calculateDistance(List<CellPoint> path, int feetPerCell) {
		if (path == null || path.size() == 0)
			return 0;

		final int feetDistance;
		{
			int numDiag = 0;
			int numStrt = 0;

			CellPoint previousPoint = null;
			for (CellPoint point : path) {
				if (previousPoint != null) {
					int change = Math.abs(previousPoint.x - point.x) + Math.abs(previousPoint.y - point.y);

					switch (change) {
					case 1:
						numStrt++;
						break;
					case 2:
						numDiag++;
						break;
					default:
						assert false : String.format("Illegal path, cells are not contiguous; change=%d", change);
						return -1;
					}
				}
				previousPoint = point;
			}
			final int cellDistance;
			switch (metric) {
			case MANHATTAN:
			case NO_DIAGONALS:
				cellDistance = (numStrt + numDiag * 2);
				break;
			case ONE_ONE_ONE:
				cellDistance = (numStrt + numDiag);
				break;
			default:
			case ONE_TWO_ONE:
				cellDistance = (numStrt + numDiag + numDiag / 2);
				break;
			}
			feetDistance = feetPerCell * cellDistance;
		}
		return feetDistance;
	}
}
