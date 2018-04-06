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
import java.awt.geom.Area;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.locationtech.jts.awt.ShapeReader;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;

import net.rptools.maptool.client.MapTool;
import net.rptools.maptool.client.walker.WalkerMetric;
import net.rptools.maptool.model.CellPoint;
import net.rptools.maptool.model.Token;
import net.rptools.maptool.model.Zone;

public class AStarSquareEuclideanWalker extends AbstractAStarWalker {
	private static final Logger log = LogManager.getLogger(AStarSquareEuclideanWalker.class);

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

	private double diagonal_mult_real = 1;
	private double diagonal_mult_game = 1;

	private List<Token> terrainTokens;
	List<AStarCellPoint> terrainCells = new ArrayList<AStarCellPoint>();

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
			diagonal_mult_real = 1.414;
			diagonal_mult_game = 1.5;
			neighborMap = new int[][] { NORTH, EAST, SOUTH, WEST, NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST };
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

		// Get tokens on map that may affect movement
		terrainTokens = zone.getTokensWithTerrainModifiers();
		for (Token token : terrainTokens) {
			// log.info("Token: " + token.getName() + ", " + token.getTerrainModifier());
			Set<CellPoint> cells = token.getOccupiedCells(zone.getGrid());
			for (CellPoint cell : cells)
				terrainCells.add(new AStarCellPoint(cell, token.getTerrainModifier()));
		}

	}

	@Override
	protected List<AStarCellPoint> getNeighbors(AStarCellPoint node, Set<AStarCellPoint> closedSet) {
		List<AStarCellPoint> neighbors = new ArrayList<AStarCellPoint>();
		int[][] neighborMap = getNeighborMap(node.x, node.y);

		// Find all the neighbors.
		for (int[] i : neighborMap) {
			double terrainModifier = 0;

			AStarCellPoint neighbor = new AStarCellPoint(node.x + i[0], node.y + i[1]);

			if (closedSet.contains(neighbor))
				continue;

			// Add the cell we're coming from
			neighbor.parent = node;

			// restrictMovement = false; // TURN OFF AI
			// Don't count VBL or Terrain Modifiers
			if (restrictMovement) {
				// VBL Check FIXME: Add to closed set?
				if (vblBlocksMovement(node, neighbor)) {
					closedSet.add(node);
					continue;
				}

				// FIXME: add Occupied cell!
				// Check for terrain modifiers
				for (AStarCellPoint cell : terrainCells) {
					if (cell.equals(neighbor)) {
						terrainModifier += cell.terrainModifier;
						// log.info("terrainModifier for " + cell + " = " + cell.terrainModifier);
					}
				}
			}

			if (terrainModifier == 0)
				terrainModifier = 1;

			// Simple check if it's a diagonal neighbor
			if (Arrays.equals(i, NORTH_EAST) || Arrays.equals(i, SOUTH_EAST) || Arrays.equals(i, SOUTH_WEST) || Arrays.equals(i, NORTH_WEST)) {
				neighbor.g = node.g + (normal_cost * terrainModifier * diagonal_mult_real);
				neighbor.distanceTraveled = node.distanceTraveled + (normal_cost * terrainModifier * diagonal_mult_game);
			} else {
				neighbor.g = node.g + (normal_cost * terrainModifier);
				neighbor.distanceTraveled = node.distanceTraveled + (normal_cost * terrainModifier);
			}

			neighbors.add(neighbor);
			// log.info("neighbor.g: " + neighbor.getG());
		}

		return neighbors;
	}

	private boolean vblBlocksMovement(AStarCellPoint start, AStarCellPoint goal) {
		if (vblGeometry == null)
			return false;

		Rectangle startBounds = zone.getGrid().getBounds(start);
		Rectangle goalBounds = zone.getGrid().getBounds(goal);

		if (goalBounds.isEmpty() || startBounds.isEmpty())
			return false;

		// If there is no vbl within the footprints, we're good!
		if (!vbl.intersects(startBounds) && !vbl.intersects(goalBounds))
			return false;

		double x1 = startBounds.getCenterX();
		double y1 = startBounds.getCenterY();
		double x2 = goalBounds.getCenterX();
		double y2 = goalBounds.getCenterY();

		// If the goal center point is in vbl, allow to maintain path through vbl (should be GM only?)
		if (vbl.contains(goal.toPoint())) {
			// Allow GM to move through VBL
			return !MapTool.getPlayer().isGM();
		}

		// Ok, now we have to check from cell to cell for partial vbl.
		// Basically, we will cast a few rays and see if the previous cell can "see" the new cell...
		// For now, we'll assume center to center?

		LineString centerRay = geometryFactory.createLineString(new Coordinate[] { new Coordinate(x1, y1), new Coordinate(x2, y2) });
		// log.info("centerRay: " + centerRay);

		boolean blocksMovement;
		try {
			// double clipBuffer = Math.max(size / 2, metaBuffer) + 10;
			// Envelope env = new Envelope(x, y, x + w, y + h);
			// env.expandBy(1); do we need a buffer? probably...
			// Geometry clip = geometryFactory.toGeometry(env);
			// Geometry clipped = vblGeometry.intersection(clip);

			blocksMovement = vblGeometry.intersects(centerRay);
			// log.info("blocksMovement: " + blocksMovement);
		} catch (Exception e) {
			log.info("clipped.intersects oh oh: ", e);
			return true;
		}

		// log.info("Time to test: " + (System.currentTimeMillis() - startTimer));

		return blocksMovement;
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
			xDist = Math.abs(node.x - goal.x);
			yDist = Math.abs(node.y - goal.y);
			if (xDist > yDist)
				distance = diagonal_mult_real * yDist + (xDist - yDist);
			else
				distance = diagonal_mult_real * xDist + (yDist - xDist);
			break;
		case ONE_ONE_ONE:
			distance = Math.max(Math.abs(xDist), Math.abs(yDist));
			break;
		}

		return distance;
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

	@Override
	protected int calculateDistance(List<CellPoint> path, int feetPerCell) {
		if (path == null || path.size() == 0)
			return 0;

		log.info("Should not see this! ");
		System.exit(9000);

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
