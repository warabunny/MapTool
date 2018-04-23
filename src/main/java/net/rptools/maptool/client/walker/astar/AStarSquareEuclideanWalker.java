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
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;

import com.google.common.base.Stopwatch;

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
		int[][] neighborMap = getNeighborMap(node.x, node.y); // FIXME: Larger map needed for larger tokens!

		// Find all the neighbors.
		for (int[] i : neighborMap) {
			double terrainModifier = 0;
			boolean blockNode = false;

			AStarCellPoint neighbor = new AStarCellPoint(node.x + i[0], node.y + i[1]);
			Set<CellPoint> occupiedCells = footprint.getOccupiedCells(node);
			// Rectangle footprintBounds = footprint.getBounds(zone.getGrid(), node);
			// log.info("footprintBounds: " + footprintBounds);

			if (closedSet.contains(neighbor))
				continue;

			// Add the cell we're coming from
			neighbor.parent = node;

			// restrictMovement = false; // TURN OFF AI
			// Don't count VBL or Terrain Modifiers
			if (restrictMovement) {
				// log.info("footprint: " + footprint.getOccupiedCells(node));
				for (CellPoint cellPoint : occupiedCells) {
					AStarCellPoint occupiedNode = new AStarCellPoint(cellPoint);

					// FIXME: This is slow as shit for huge+
					// log.info("node: " + node);
					// log.info("occupiedNode: " + occupiedNode);
					// log.info("neighbor: " + neighbor);

					// VBL Check FIXME: Add to closed set?
					if (vblBlocksMovement(occupiedNode, neighbor)) {
						closedSet.add(occupiedNode);
						blockNode = true;
						break;
					}

				}

				// if (vblBlocksMovement(node, neighbor)) {
				// closedSet.add(node);
				// blockNode = true;
				// //continue; // need?
				// }

				if (blockNode)
					continue;

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
		// MapTool.getFrame().getCurrentZoneRenderer().setShape(null);
		// MapTool.getFrame().getCurrentZoneRenderer().setShape2(null);

		if (vblGeometry == null)
			return false;

		Stopwatch stopwatch = Stopwatch.createStarted();
		AStarCellPoint checkNode = checkedList.get(goal);
		if (checkNode != null) {
			Boolean test = checkNode.isValidMove(start);
			
			// if it's null then the test for that direction hasn't been set yet otherwise just return the previous result
			if(test != null) {
				// log.info("Time to retrieve: " + stopwatch.elapsed(TimeUnit.NANOSECONDS));
				avgRetrieveTime += stopwatch.elapsed(TimeUnit.NANOSECONDS);
				retrievalCount++;
				return test;
			} else {
				// Copies all previous checks to save later...
				goal = checkNode;
			}
		}

		Rectangle startBounds = zone.getGrid().getBounds(start);
		Rectangle goalBounds = zone.getGrid().getBounds(goal);

		// Rectangle startBounds = footprint.getBounds(zone.getGrid(), start);
		// Rectangle goalBounds = footprint.getBounds(zone.getGrid(), goal);
		// log.info("startbounds: " + startBounds);
		// log.info("goalbounds: " + goalBounds);

		if (goalBounds.isEmpty() || startBounds.isEmpty())
			return false;

		// If there is no vbl within the footprints, we're good!
		if (!vbl.intersects(startBounds) && !vbl.intersects(goalBounds))
			return false;

		// log.info("start: " + start);
		// log.info("goal: " + goal);

		// If the goal center point is in vbl, allow to maintain path through vbl (should be GM only?)
		if (vbl.contains(goal.toPoint())) {
			// Allow GM to move through VBL
			// return !MapTool.getPlayer().isGM();
		}

		// NEW WAY - use polygon test
		int gridOffset = zone.getGrid().getSize() / 2;
		// gridOffset = 0;

		// // just check the end goal?
		// Path2D goalPath = new Path2D.Double();
		// goalPath.moveTo(goalBounds.getX() + gridOffset, goalBounds.getY() + gridOffset);
		// goalPath.lineTo(goalBounds.getMaxX() - gridOffset, goalBounds.getMaxY() - gridOffset);
		// goalPath.lineTo(goalBounds.getMaxX() - gridOffset, goalBounds.getMaxY() - gridOffset);
		// goalPath.lineTo(goalBounds.getX() + gridOffset, goalBounds.getY() + gridOffset);
		// goalPath.closePath();
		//
		// Area goalArea = new Area(goalPath);
		// goalArea.intersect(vbl);
		// if (!goalArea.isEmpty())
		// return true;
		//
		//
		// // cross the path test?
		// Path2D travelPath = new Path2D.Double();
		// travelPath.moveTo(startBounds.getX() + gridOffset, startBounds.getY() + gridOffset);
		// travelPath.lineTo(startBounds.getMaxX() - gridOffset, startBounds.getMaxY() - gridOffset);
		// travelPath.lineTo(goalBounds.getMaxX() - gridOffset, goalBounds.getMaxY() - gridOffset);
		// travelPath.lineTo(goalBounds.getX() + gridOffset, goalBounds.getY() + gridOffset);
		// travelPath.closePath();
		//
		// Area travelArea = new Area(travelPath);
		// log.info("travelArea: " + travelPath.getBounds());

		// single cell ray test
		// Path2D travelPath = new Path2D.Double();
		// travelPath.moveTo(startBounds.getX() + gridOffset, startBounds.getY() + gridOffset);
		// travelPath.lineTo(goalBounds.getX() + gridOffset, goalBounds.getY() + gridOffset);
		// travelPath.lineTo(goalBounds.getX() + 1 + gridOffset, goalBounds.getY() + 1 + gridOffset);
		// travelPath.closePath();
		// Area travelArea = new Area(travelPath);

		// MapTool.getFrame().getCurrentZoneRenderer().drawText("YO!", startBounds.x, startBounds.y);
		// MapTool.getFrame().getCurrentZoneRenderer().drawShape(travelPath, startBounds.x, startBounds.y);
		// MapTool.getFrame().getCurrentZoneRenderer().setShape(travelPath);

		// travelArea.intersect(vbl);

		// Shape test = MapTool.getFrame().getCurrentZoneRenderer().getShape2();
		// Area a = new Area(test);
		// a.add(travelArea);
		// MapTool.getFrame().getCurrentZoneRenderer().setShape2(a);

		// return !travelArea.isEmpty();

		// Ok, now we have to check from cell to cell for partial vbl.
		// Basically, we will cast a few rays and see if the previous cell can "see" the new cell...
		// For now, we'll assume center to center?

		// OLD WAY
		

		double x1 = startBounds.getCenterX();
		double y1 = startBounds.getCenterY();
		double x2 = goalBounds.getCenterX();
		double y2 = goalBounds.getCenterY();
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

		//log.info("Time to test: " + stopwatch.elapsed(TimeUnit.NANOSECONDS));
		avgTestTime += stopwatch.elapsed(TimeUnit.NANOSECONDS);
		testCount++;
		
		goal.setValidMove(start, blocksMovement);
		checkedList.put(goal, goal);

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
