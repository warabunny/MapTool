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
import java.awt.geom.Point2D;

import net.rptools.maptool.model.CellPoint;
import net.rptools.maptool.model.Zone;

public class AStarCellPoint extends CellPoint implements Comparable<AStarCellPoint> {
	AStarCellPoint parent;
	double h;
	double f;
	double terrainModifier;

	public AStarCellPoint() {
		super(0, 0);
	}

	public AStarCellPoint(int x, int y) {
		super(x, y);
	}

	public AStarCellPoint(CellPoint p) {
		super(p.x, p.y, p.distanceTraveled);
	}

	public AStarCellPoint(CellPoint p, double mod) {
		super(p.x, p.y);
		terrainModifier = mod;
	}

	public double cost() {
		return h + getG();
	}

	public Point2D toPoint() {
		return new Point2D.Double(x, y);
	}

	public Point2D toCenterPoint(Zone zone) {
		Rectangle bounds = zone.getGrid().getBounds(this);
		return new Point2D.Double(bounds.getCenterX(), bounds.getCenterY());
	}

	@Override
	public int compareTo(AStarCellPoint other) {
		return Double.compare(f, other.f);
	}
}
