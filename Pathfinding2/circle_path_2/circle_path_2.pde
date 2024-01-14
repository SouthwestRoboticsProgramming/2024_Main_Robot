// Known issue: Path arcs can sometimes pass through obstacles if the inflated obstacle
// arcs overlap, won't fix since the problem doesn't happen on the 2024 field

// To Do:
// Precompute visibility graph (BIG)
// Use bidirectional Dijkstra's algorithm (BIG)
// Rewrite in Rust
// Use a heuristic somehow to make it A*
// Node can be uniquely represented by its own and its previous arc contexts, tree could collapse back on itself for less overall search
// Use trig identities/lookup tables to speed up tangent calculations

import java.util.*;

// Environment representation: as basic shapes, in field space

interface Obstacle {
  void convertInto(float inflate, List<Arc> arcs, List<Segment> segments);
  
  void draw();
}

final class Circle implements Obstacle {
  public final float x, y;
  public final float radius;
  
  public Circle(float x, float y, float radius) {
    this.x = x;
    this.y = y;
    this.radius = radius;
  }
  
  @Override
  public void convertInto(float inflate, List<Arc> arcs, List<Segment> segments) {
    arcs.add(new Arc(x, y, radius + inflate, -PI, PI));
  }
  
  @Override
  public void draw() {
    ellipseMode(CENTER);
    ellipse(x, y, radius*2, radius*2);
  }
}

static final class Vec2 {
  public final float x;
  public final float y;
  
  public Vec2(float x, float y) {
    this.x = x;
    this.y = y;
  }
}

public Polygon createRectangle(float x, float y, float width, float height, float rotation) {
    float halfW = width/2;
    float halfH = height/2;
    
    float sin = sin(rotation);
    float cos = cos(rotation);
    
    List<Vec2> vertices = new ArrayList<>();
    vertices.add(new Vec2(x + -halfW * cos - -halfH * sin, y + -halfW * sin + -halfH * cos));
    vertices.add(new Vec2(x + halfW * cos - -halfH * sin, y + halfW * sin + -halfH * cos));
    vertices.add(new Vec2(x + halfW * cos - halfH * sin, y + halfW * sin + halfH * cos));
    vertices.add(new Vec2(x + -halfW * cos - halfH * sin, y + -halfW * sin + halfH * cos));
    
    return new Polygon(vertices);
  }

// If vertices are wound ccw, inside will be filled
// cw, outside will be filled
class Polygon implements Obstacle {
  private final List<Vec2> vertices;
  
  public Polygon(List<Vec2> vertices) {
    this.vertices = vertices;
  }
  
  @Override
  public void convertInto(float inflate, List<Arc> arcs, List<Segment> segments) {
    Vec2 prev = vertices.get(vertices.size() - 1);
    for (int i = 0; i < vertices.size(); i++) {
      Vec2 vertex = vertices.get(i);
      float dx = vertex.x - prev.x;
      float dy = vertex.y - prev.y;
      float d = sqrt(dx * dx + dy * dy);
      
      float s = inflate / d;
      float ox = dy * s;
      float oy = -dx * s;
      
      segments.add(new Segment(vertex.x + ox, vertex.y + oy, prev.x + ox, prev.y + oy));
      
      Vec2 next = vertices.get((i + 1) % vertices.size());
      float edgeAngle = atan2(dy, dx);
      float nextAngle = atan2(next.y - vertex.y, next.x - vertex.x);
      if (floorMod(nextAngle - edgeAngle, TWO_PI) < PI) {
        arcs.add(new Arc(vertex.x, vertex.y, inflate, edgeAngle - HALF_PI, nextAngle - HALF_PI));
      }
      
      prev = vertex;
    }
  }
  
  @Override
  public void draw() {
    beginShape();
    for (Vec2 vertex : vertices) {
      vertex(vertex.x, vertex.y);
    }
    endShape(CLOSE);
  }
}

// Pathfinding representation: as arcs and segments
// All paths will be tangent to only arcs, and will never intersect any segments or arcs

static final class Arc {
  public final float centerX, centerY;
  public final float radius;
  public final float angleMin, angleMax;
  
  public Arc(float centerX, float centerY, float radius, float angleMin, float angleMax) {
    this.centerX = centerX;
    this.centerY = centerY;
    this.radius = radius;
    this.angleMin = wrapAngle(angleMin);
    this.angleMax = wrapAngle(angleMax);
  }
  
  public boolean containsAngle(float angle) {
    if (angleMin == angleMax)
      return true;
    
    angle = wrapAngle(angle);
    
    float relMax = angleMax;
    if (relMax < angleMin) relMax += TWO_PI;
    if (angle < angleMin) angle += TWO_PI;
    
    return angle >= angleMin && angle <= relMax;
  }
}

static final class Segment {
  public final float x1, y1;
  public final float x2, y2;
  
  public Segment(float x1, float y1, float x2, float y2) {
    this.x1 = x1;
    this.y1 = y1;
    this.x2 = x2;
    this.y2 = y2;
  }
  
  public float length() {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
  }
}

// --------------------------------

enum WindingDir {
  Clockwise,
  Counterclockwise;
  
  static {
    Clockwise.opposite = Counterclockwise;
    Counterclockwise.opposite = Clockwise;
  }
  
  private WindingDir opposite;
  
  public WindingDir opposite() {
    return opposite;
  }
}

static final class PointToArcTangent {
  public final Segment segment;
  public final float arcAngle;
  public final WindingDir arcDir;
  
  public PointToArcTangent(Segment segment, float arcAngle, WindingDir arcDir) {
    this.segment = segment;
    this.arcAngle = arcAngle;
    this.arcDir = arcDir;
  }
}

static final class ArcToArcTangent {
  public final Segment segment;
  public final float fromAngle, toAngle;
  public final WindingDir arcDir;
  
  public ArcToArcTangent(Segment segment, float fromAngle, float toAngle, WindingDir arcDir) {
    this.segment = segment;
    this.fromAngle = fromAngle;
    this.toAngle = toAngle;
    this.arcDir = arcDir;
  }
}

boolean segmentPassable(Segment seg, Arc... ignore) {
  loop: for (Arc arc : arcs) {
    for (Arc i : ignore)
      if (arc == i)
        continue loop;
    
    float dist = segmentDistSq(
      arc.centerX, arc.centerY,
      seg.x1, seg.y1,
      seg.x2, seg.y2);
      
    if (dist < arc.radius * arc.radius)
      return false;
  }
  
  for (Segment segment : segments) {
    float dx1 = segment.x2 - segment.x1;
    float dy1 = segment.y2 - segment.y1;
    float dx2 = seg.x2 - seg.x1;
    float dy2 = seg.y2 - seg.y1;
    
    float vp = dx1 * dy2 - dx2 * dy1;
    if (abs(vp) < 0.01)
      continue; // Ignore collision if the segments are collinear
      
    float vx = seg.x1 - segment.x1;
    float vy = seg.y1 - segment.y1;
    
    float k1 = (vx * dy2 - vy * dx2) / vp;
    if (k1 < 0 || k1 > 1)
      continue;
      
    float k2 = (vx * dy1 - vy * dx1) / vp;
    if (k2 < 0 || k2 > 1)
      continue;
    
    return false;
  }
  
  return true;
}

List<PointToArcTangent> findPointToArcTangents(float px, float py, Arc arc) {
  float dx = px - arc.centerX;
  float dy = py - arc.centerY;
  float distance = sqrt(dx * dx + dy * dy);
  
  float angleOffset = acos(arc.radius / distance);
  float baseAngle = atan2(dy, dx);
  
  float cwAngle = baseAngle - angleOffset;
  float ccwAngle = baseAngle + angleOffset;
  
  Segment cw = new Segment(
    px, py,
    arc.centerX + arc.radius * cos(cwAngle),
    arc.centerY + arc.radius * sin(cwAngle)
  );
  Segment ccw = new Segment(
    px, py,
    arc.centerX + arc.radius * cos(ccwAngle),
    arc.centerY + arc.radius * sin(ccwAngle)
  );
  
  List<PointToArcTangent> out = new ArrayList<>();
  if (arc.containsAngle(cwAngle) && segmentPassable(cw, arc))
    out.add(new PointToArcTangent(cw, cwAngle, WindingDir.Clockwise));
  if (arc.containsAngle(ccwAngle) && segmentPassable(ccw, arc))
    out.add(new PointToArcTangent(ccw, ccwAngle, WindingDir.Counterclockwise));
  return out;
}

List<ArcToArcTangent> findArcToArcTangents(Arc from, Arc to, WindingDir fromDir) {
  float dx = to.centerX - from.centerX;
  float dy = to.centerY - from.centerY;
  float distance = sqrt(dx * dx + dy * dy);
  float intersectDist = distance * from.radius / (from.radius + to.radius);
  
  float baseAngle = atan2(dy, dx);
 
  boolean isCw = fromDir == WindingDir.Clockwise;
  float angleFlip = isCw ? 1 : -1;
  float sameDir = baseAngle + angleFlip * (HALF_PI + asin((to.radius - from.radius) / distance));
  float crossDir = baseAngle + angleFlip * acos(from.radius / intersectDist);
  
  List<ArcToArcTangent> out = new ArrayList<>();
  
  if (from.containsAngle(sameDir) && to.containsAngle(sameDir)) {
    float sinSame = sin(sameDir);
    float cosSame = cos(sameDir);
    
    Segment same = new Segment(
      from.centerX + from.radius * cosSame,
      from.centerY + from.radius * sinSame,
      to.centerX + to.radius * cosSame,
      to.centerY + to.radius * sinSame
    );
    
    if (segmentPassable(same, from, to))
      out.add(new ArcToArcTangent(same, sameDir, sameDir, fromDir));
  }
  
  if (from.containsAngle(crossDir) && to.containsAngle(crossDir + PI)) {
    float sinCross = sin(crossDir);
    float cosCross = cos(crossDir);
    
    Segment cross = new Segment(
      from.centerX + from.radius * cosCross,
      from.centerY + from.radius * sinCross,
      to.centerX - to.radius * cosCross,
      to.centerY - to.radius * sinCross
    );
    
    if (segmentPassable(cross, from, to))
      out.add(new ArcToArcTangent(cross, crossDir, crossDir + PI, fromDir.opposite()));
  }
  
  return out;
}

// --------------------------------

static final class ArcContext {
  public final Arc arc;
  public final WindingDir dir;
  
  public ArcContext(Arc arc, WindingDir dir) {
    this.arc = arc;
    this.dir = dir;
  }
  
  @Override
  public boolean equals(Object o) {
    if (o == null) return false;
    if (o == this) return true;
    if (!o.getClass().equals(getClass())) return false;
    
    ArcContext c = (ArcContext) o;
    return arc == c.arc && dir == c.dir;
  }
  
  @Override
  public int hashCode() {
    return Objects.hash(arc, dir);
  }
}

static final class PathNode implements Comparable<PathNode> {
  public final ArcContext context;
  
  public PathNode cameFrom = null;
  public float incomingAngle, parentOutgoingAngle;
  
  public float costSoFar;
  public boolean isGoal = false;
  
  Segment debugSegment;
  
  public PathNode(ArcContext context) {
    this.context = context;
  }
  
  public boolean hasVisited(ArcContext context) {
    if (context.equals(this.context))
      return true;
    return cameFrom != null && cameFrom.hasVisited(context);
  }
  
  @Override
  public int compareTo(PathNode other) {
    return Float.compare(costSoFar, other.costSoFar);
  }
}

float calcTurnCost(float incomingAngle, float outgoingAngle, float radius, WindingDir dir) {
  float diff;
  if (dir == WindingDir.Counterclockwise)
    diff = outgoingAngle - incomingAngle;
  else
    diff = incomingAngle - outgoingAngle;
  diff = floorMod(diff, TWO_PI);
  
  return diff * radius;
}

static final class PathArc {
  float centerX, centerY;
  float radius;
  float incomingAngle, outgoingAngle;
  WindingDir dir;
  
  public PathArc(float centerX, float centerY, float radius, float incomingAngle, float outgoingAngle, WindingDir dir) {
    this.centerX = centerX;
    this.centerY = centerY;
    this.radius = radius;
    this.incomingAngle = incomingAngle;
    this.outgoingAngle = outgoingAngle;
    this.dir = dir;
  }
}

// Returns the arcs the path goes along
// They should be connected between with straight line, and connected to start and goal at ends
List<PathArc> findPath(float startX, float startY, float goalX, float goalY) {
  Segment direct = new Segment(startX, startY, goalX, goalY);
  if (segmentPassable(direct))
    return Collections.emptyList();
  
  PriorityQueue<PathNode> frontier = new PriorityQueue<>();
  
  for (Arc arc : arcs) {
    for (PointToArcTangent t : findPointToArcTangents(startX, startY, arc)) {
      ArcContext ctx = new ArcContext(arc, t.arcDir);
      PathNode node = new PathNode(ctx);
      node.costSoFar = t.segment.length();
      node.incomingAngle = t.arcAngle;
      
      node.debugSegment = t.segment;
      
      frontier.add(node);
    }
  }
  
  while (!frontier.isEmpty()) {
    PathNode current = frontier.remove();
    
    if (current.debugSegment != null) {
      Segment segment = current.debugSegment;
      strokeWeight(2);
          stroke(0, 128, 255);
          line(segment.x1, segment.y1, segment.x2, segment.y2);
    }
    
    if (current.isGoal) {
      List<PathArc> out = new ArrayList<>();
      while (true) {
        float outgoing = current.parentOutgoingAngle;
        current = current.cameFrom;
        if (current == null)
          break;
          
        float incoming = current.incomingAngle;
        Arc arc = current.context.arc;
        out.add(0, new PathArc(arc.centerX, arc.centerY, arc.radius, incoming, outgoing, current.context.dir));
      }
      
      return out;
    }
    
    // key: from arc, direction
    // values: arc to arc tangents for each arc
    
    for (Arc arc : arcs) {
      if (arc == current.context.arc) continue;
      
      for (ArcToArcTangent t : findArcToArcTangents(current.context.arc, arc, current.context.dir)) {
        ArcContext ctx = new ArcContext(arc, t.arcDir);
        
        if (!current.hasVisited(ctx)) {
          float distanceCost = t.segment.length();
          float turnCost = calcTurnCost(current.incomingAngle, t.fromAngle, current.context.arc.radius, current.context.dir);
          
          PathNode node = new PathNode(ctx);
          node.costSoFar = current.costSoFar + distanceCost + turnCost;
          node.cameFrom = current;
          node.incomingAngle = t.toAngle;
          node.parentOutgoingAngle = t.fromAngle;
          
          node.debugSegment = t.segment;
          
          frontier.add(node);
        }
      }
    }
    
    for (PointToArcTangent t : findPointToArcTangents(goalX, goalY, current.context.arc)) {
      // Note: This means we are effectively reversing the direction, since we're taking
      // the goal tangent in reverse
      if (current.context.dir == t.arcDir)
        continue;
        
      float distanceCost = t.segment.length();
      float turnCost = calcTurnCost(current.incomingAngle, t.arcAngle, current.context.arc.radius, current.context.dir);
      
      PathNode node = new PathNode(null);
      node.costSoFar = current.costSoFar + distanceCost + turnCost;
      node.cameFrom = current;
      node.isGoal = true;
      node.parentOutgoingAngle = t.arcAngle;
      
      frontier.add(node);
    }
  }
  
  println("Nope");
  return null;
}

// --------------------------------

private final List<Obstacle> obstacles = new ArrayList<>();

private final List<Arc> arcs = new ArrayList<>();
private final List<Segment> segments = new ArrayList<>();

private final float inflate = 50;

float startX = 100;
float startY = 100;
float goalX = 150;
float goalY = 150;

void settings() {
  size(800, 600);
}

void genObstacles() {
  obstacles.clear();
  arcs.clear();
  segments.clear();
  //for (int i = 0; i < 10; i++) {
  //  obstacles.add(createRectangle(random(50, width-50), random(50, height-50), 10, 10, random(0, 2*PI)));
  //  obstacles.add(new Circle(random(50, width-50), random(50, height-50), random(5, 50)));
  //}
  //obstacles.add(new Circle(200, 300, 50));
  //obstacles.add(createRectangle(500, 300, 10, 10, QUARTER_PI));
  //for (int i = 0; i < 10; i++) {
  //  obstacles.add(createRectangle(100 + i * 100, 300 + (i % 2) * 200, 10, 100, 0));
  //}
  
  obstacles.add(new Polygon(List.of(new Vec2(0, 0), new Vec2(0, height/3), new Vec2(50, height/3+50), new Vec2(50, 2*height/3 - 50), new Vec2(0, 2*height/3), new Vec2(0, height-50), new Vec2(100, height),
  new Vec2(width - 100, height), new Vec2(width, height-50), new Vec2(width, 2*height/3), new Vec2(width-50, 2*height/3 - 50), new Vec2(width-50, height/3+50), new Vec2(width, height/3), new Vec2(width, 0))));
  
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      obstacles.add(createRectangle(width/2 + (i-1) * 200, height/2 + (j-0.5f) * 150, 40, 40, 0));
    }
  }
  
  for (Obstacle o : obstacles)
    o.convertInto(inflate, arcs, segments);
}

void setup() {
  genObstacles();
}

void keyPressed() {
  if (key == ' ') genObstacles();
}

void mouseDragged() {
  if (mouseButton == LEFT) {
    startX = mouseX;
    startY = height - mouseY;
  } else {
    goalX = mouseX;
    goalY = height-mouseY;
  }
}

void draw() {
  translate(width/2, height/2);
  scale(1, -1);
  translate(-width/2, -height/2);
  
  background(0);
  
  strokeWeight(1);
  stroke(255, 128, 0);
  noFill();
  for (Obstacle o : obstacles)
    o.draw();
    
  stroke(128);
  for (Segment s : segments)
    line(s.x1, s.y1, s.x2, s.y2);
  stroke(255, 0, 0);
  for (Arc arc : arcs) {
    if (arc.angleMin == arc.angleMax) {
      ellipse(arc.centerX, arc.centerY, arc.radius * 2, arc.radius * 2);
      continue;
    }
    float maxAngle = arc.angleMax;
    if (maxAngle < arc.angleMin)
      maxAngle += TWO_PI;
    arc(arc.centerX, arc.centerY, arc.radius*2, arc.radius*2, arc.angleMin, maxAngle);
  }
  
  List<PathArc> path = findPath(startX, startY, goalX, goalY);
  if (path != null) {
    strokeWeight(6);
    stroke(0, 255, 0);
    
    float px = startX, py = startY;
    for (PathArc arc : path) {
      line(px, py, arc.centerX + arc.radius * cos(arc.incomingAngle), arc.centerY + arc.radius * sin(arc.incomingAngle));
      
      float angle1 = wrapAngle(arc.incomingAngle);
      float angle2 = wrapAngle(arc.outgoingAngle);
      if (arc.dir == WindingDir.Clockwise) {
        float temp = angle1;
        angle1 = angle2;
        angle2 = temp;
      }
      
      if (angle2 < angle1) angle2 += TWO_PI;
      
      
      arc(arc.centerX, arc.centerY, arc.radius*2, arc.radius*2, angle1, angle2);
      px = arc.centerX + arc.radius * cos(arc.outgoingAngle);
      py = arc.centerY + arc.radius * sin(arc.outgoingAngle);
    }
    line(px, py, goalX, goalY);
  }
}

public static float floorMod(float x, float y) {
  if (y == 0) throw new ArithmeticException("Divide by zero");

  return x - floor(x / y) * y;
}

// Put angle into -PI to PI
static float wrapAngle(float angle) {
  return floorMod(angle + PI, TWO_PI) - PI;
}

float distanceSq(float x1, float y1, float x2, float y2) {
  float dx = x1 - x2;
  float dy = y1 - y2;
  return dx * dx + dy * dy;
}

float segmentDistSq(float px, float py, float x1, float y1, float x2, float y2) {
  float l2 = distanceSq(x1, y1, x2, y2);
  if (l2 == 0)
    return distanceSq(px, py, x1, y1);
  float t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / l2;
  t = constrain(t, 0, 1);
  return distanceSq(
    px, py,
    lerp(x1, x2, t),
    lerp(y1, y2, t));
}
