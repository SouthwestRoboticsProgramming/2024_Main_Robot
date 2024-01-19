use std::{cmp::Ordering, collections::BinaryHeap, f64::consts::PI, rc::Rc};

use arrayvec::ArrayVec;
use lerp::Lerp;
use ordered_float::OrderedFloat;

use crate::{
    math::{self, Vec2f},
    obstacle::Obstacle,
};

pub struct Arc {
    pub center: Vec2f,
    pub radius: f64,
    pub min_angle: f64,
    pub max_angle: f64,
}

impl Arc {
    pub fn new(center: Vec2f, radius: f64, min_angle: f64, max_angle: f64) -> Self {
        Self {
            center,
            radius,
            min_angle: math::wrap_angle(min_angle),
            max_angle: math::wrap_angle(max_angle),
        }
    }

    pub fn contains_angle(&self, angle: f64) -> bool {
        if self.min_angle == self.max_angle {
            return true;
        }

        let mut angle = math::wrap_angle(angle);

        let mut rel_max = self.max_angle;
        if rel_max < self.min_angle {
            rel_max += PI * 2.0;
        }
        if angle < self.min_angle {
            angle += PI * 2.0;
        }

        angle >= self.min_angle && angle <= rel_max
    }
}

pub struct Segment {
    pub from: Vec2f,
    pub to: Vec2f,
}

impl Segment {
    pub fn length(&self) -> f64 {
        (self.to - self.from).length()
    }
}

// -------------------------

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum WindingDir {
    Clockwise,
    Counterclockwise,
}

impl WindingDir {
    pub fn opposite(self) -> Self {
        match self {
            Self::Clockwise => Self::Counterclockwise,
            Self::Counterclockwise => Self::Clockwise,
        }
    }
}

pub struct PointToArcTangent {
    pub segment: Segment,
    pub arc_angle: f64,
    pub arc_dir: WindingDir,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct ArcContext(pub usize);

impl ArcContext {
    pub fn new(id: usize, dir: WindingDir) -> Self {
        Self(
            id << 1
                | match dir {
                    WindingDir::Clockwise => 0,
                    WindingDir::Counterclockwise => 1,
                },
        )
    }

    pub fn arc_id(self) -> usize {
        self.0 >> 1
    }

    pub fn dir(self) -> WindingDir {
        match self.0 & 1 {
            0 => WindingDir::Clockwise,
            1 => WindingDir::Counterclockwise,
            _ => unreachable!(),
        }
    }
}

pub struct VisibilityEdge {
    pub segment: Segment,
    pub from_angle: f64,
    pub to_angle: f64,
    pub dest: ArcContext,
}

#[derive(Clone)]
pub struct PathArc {
    pub center: Vec2f,
    pub radius: f64,
    pub incoming_angle: f64,
    pub outgoing_angle: f64,
    pub direction: WindingDir,
}

struct SearchNode {
    pub context: ArcContext,
    pub is_goal: bool,

    // TODO: Should probably not allocate every node on heap
    pub came_from: Option<Rc<SearchNode>>,
    pub incoming_angle: f64,
    pub parent_outgoing_angle: f64,

    pub cost_so_far: f64,
}

impl SearchNode {
    pub fn has_visited(&self, context: ArcContext) -> bool {
        if context == self.context {
            return true;
        }

        match &self.came_from {
            Some(node) => node.has_visited(context),
            None => false,
        }
    }
}

impl Ord for SearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        OrderedFloat(self.cost_so_far)
            .cmp(&OrderedFloat(other.cost_so_far))
            .reverse() // Turns the heap into a minheap
    }
}

impl PartialOrd for SearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for SearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for SearchNode {}

pub struct Field {
    pub arcs: Vec<Arc>,
    pub segments: Vec<Segment>,

    pub visibility: Vec<Vec<VisibilityEdge>>,
}

fn calc_turn_cost(incoming_angle: f64, outgoing_angle: f64, radius: f64, dir: WindingDir) -> f64 {
    let diff = math::floor_mod(
        match dir {
            WindingDir::Counterclockwise => outgoing_angle - incoming_angle,
            WindingDir::Clockwise => incoming_angle - outgoing_angle,
        },
        PI * 2.0,
    );

    diff * radius
}

impl Field {
    pub fn generate(obstacles: &Vec<Obstacle>, inflate: f64) -> Self {
        let mut arcs = Vec::new();
        let mut segments = Vec::new();

        for obstacle in obstacles {
            obstacle.convert_into(inflate, &mut arcs, &mut segments);
        }

        let mut field = Self {
            segments,
            visibility: Vec::with_capacity(arcs.len() * 2),
            arcs,
        };
        field.calc_visibility();

        field
    }

    pub fn find_path(&self, mut start: Vec2f, goal: Vec2f) -> Option<Vec<PathArc>> {
        if self.is_segment_passable(
            &Segment {
                from: start,
                to: goal,
            },
            None,
            None,
        ) {
            // Straight line from start to goal
            return Some(Vec::new());
        }

        // Check if the goal is unreachable, since if we try to search for unreachable
        // goal it will loop forever
        let mut tangents: ArrayVec<_, 2> = ArrayVec::new();
        let mut goal_reachable = false;
        for (arc_id, arc) in self.arcs.iter().enumerate() {
            self.find_point_to_arc_tangents(goal, arc, arc_id, &mut tangents);
            if !tangents.is_empty() {
                goal_reachable = true;
                break;
            }
        }
        if !goal_reachable {
            return None;
        }

        let mut frontier = BinaryHeap::new();

        let mut start_changed = false;
        for _ in 0..8 {
            for (arc_id, arc) in self.arcs.iter().enumerate() {
                self.find_point_to_arc_tangents(start, &arc, arc_id, &mut tangents);
                for tangent in &tangents {
                    let cost = tangent.segment.length();
                    frontier.push(Rc::new(SearchNode {
                        context: ArcContext::new(arc_id, tangent.arc_dir),
                        is_goal: false,
                        came_from: None,
                        incoming_angle: tangent.arc_angle,
                        parent_outgoing_angle: 0.0,
                        cost_so_far: cost,
                    }));
                }
            }

            if !frontier.is_empty() {
                break;
            }

            // Try to find a way back to the field
            // This is somewhat goofy, but I don't care since it really shouldn't ever happen

            let mut nearest_seg = None;
            for segment in &self.segments {
                let p1 = segment.from;
                let p2 = segment.to;

                let l2 = p1.distance_sq(p2);
                let (dist, point) = if l2 == 0.0 {
                    (start.distance_sq(p1), p1)
                } else {
                    let t =
                        ((start.x - p1.x) * (p2.x - p1.x) + (start.y - p1.y) * (p2.y - p1.y)) / l2;
                    let t = t.clamp(0.0, 1.0);

                    let pt = p1.lerp(p2, t);
                    (start.distance_sq(pt), pt)
                };

                if match nearest_seg {
                    Some((d, _)) => dist < d,
                    None => true,
                } {
                    nearest_seg = Some((dist, point));
                }
            }

            match nearest_seg {
                Some((_, proj_point)) => {
                    start = proj_point + (proj_point - start).norm() * 0.05;
                    start_changed = true;
                }
                None => return None,
            }
        }
        if frontier.is_empty() {
            // Couldn't find a way back to safe area
            return None;
        }

        // Try straight line again if it changed
        if start_changed {
            if self.is_segment_passable(
                &Segment {
                    from: start,
                    to: goal,
                },
                None,
                None,
            ) {
                // Straight line from start to goal
                return Some(Vec::new());
            }
        }

        while let Some(current) = frontier.pop() {
            if current.is_goal {
                let mut node = current;
                let mut out = Vec::new();
                loop {
                    let outgoing = node.parent_outgoing_angle;
                    match &node.came_from {
                        None => break,
                        Some(parent) => node = parent.clone(),
                    }

                    let incoming = node.incoming_angle;
                    let arc = &self.arcs[node.context.arc_id()];
                    out.push(PathArc {
                        center: arc.center,
                        radius: arc.radius,
                        incoming_angle: incoming,
                        outgoing_angle: outgoing,
                        direction: node.context.dir(),
                    });
                }

                out.reverse();

                return Some(out);
            }

            let current_arc_id = current.context.arc_id();
            let current_dir = current.context.dir();
            let current_arc = &self.arcs[current_arc_id];

            for edge in &self.visibility[current.context.0] {
                if !current.has_visited(edge.dest) {
                    let distance_cost = edge.segment.length();
                    let turn_cost = calc_turn_cost(
                        current.incoming_angle,
                        edge.from_angle,
                        current_arc.radius,
                        current.context.dir(),
                    );

                    frontier.push(Rc::new(SearchNode {
                        context: edge.dest,
                        is_goal: false,
                        came_from: Some(current.clone()),
                        incoming_angle: edge.to_angle,
                        parent_outgoing_angle: edge.from_angle,
                        cost_so_far: current.cost_so_far + distance_cost + turn_cost,
                    }));
                }
            }

            self.find_point_to_arc_tangents(goal, current_arc, current_arc_id, &mut tangents);
            for tangent in &tangents {
                if current_dir == tangent.arc_dir {
                    continue;
                }

                let distance_cost = tangent.segment.length();
                let turn_cost = calc_turn_cost(
                    current.incoming_angle,
                    tangent.arc_angle,
                    current_arc.radius,
                    current_dir,
                );

                frontier.push(Rc::new(SearchNode {
                    context: ArcContext(0), // FIXME: Restructure type so this can be omitted
                    is_goal: true,
                    came_from: Some(current.clone()),
                    incoming_angle: 0.0,
                    parent_outgoing_angle: tangent.arc_angle,
                    cost_so_far: current.cost_so_far + distance_cost + turn_cost,
                }));
            }
        }

        // Didn't find a path :(
        None
    }

    fn find_point_to_arc_tangents(
        &self,
        point: Vec2f,
        arc: &Arc,
        arc_id: usize,
        out: &mut ArrayVec<PointToArcTangent, 2>,
    ) {
        out.clear();

        let d = point - arc.center;
        let distance = d.length();

        let angle_offset = (arc.radius / distance).acos();
        let base_angle = d.angle();

        let cw_angle = base_angle - angle_offset;
        let ccw_angle = base_angle + angle_offset;

        if arc.contains_angle(cw_angle) {
            let cw = Segment {
                from: point,
                to: arc.center + Vec2f::new_angle(arc.radius, cw_angle),
            };

            if self.is_segment_passable(&cw, Some(arc_id), None) {
                out.push(PointToArcTangent {
                    segment: cw,
                    arc_angle: cw_angle,
                    arc_dir: WindingDir::Clockwise,
                });
            }
        }

        if arc.contains_angle(ccw_angle) {
            let ccw = Segment {
                from: point,
                to: arc.center + Vec2f::new_angle(arc.radius, ccw_angle),
            };

            if self.is_segment_passable(&ccw, Some(arc_id), None) {
                out.push(PointToArcTangent {
                    segment: ccw,
                    arc_angle: ccw_angle,
                    arc_dir: WindingDir::Counterclockwise,
                });
            }
        }
    }

    fn calc_visibility(&mut self) {
        // TODO: This can be optimized by only iterating half, since
        // visibility is bidirectional
        for (from_id, from_arc) in self.arcs.iter().enumerate() {
            let mut visible_cw = Vec::new();
            let mut visible_ccw = Vec::new();

            for (to_id, to_arc) in self.arcs.iter().enumerate() {
                if from_id == to_id {
                    continue;
                }

                self.find_visible_edges(
                    from_arc,
                    from_id,
                    to_arc,
                    to_id,
                    WindingDir::Clockwise,
                    &mut visible_cw,
                );
                self.find_visible_edges(
                    from_arc,
                    from_id,
                    to_arc,
                    to_id,
                    WindingDir::Counterclockwise,
                    &mut visible_ccw,
                );
            }

            self.visibility.push(visible_cw);
            self.visibility.push(visible_ccw);
        }
    }

    fn find_visible_edges(
        &self,
        from: &Arc,
        from_id: usize,
        to: &Arc,
        to_id: usize,
        from_dir: WindingDir,
        out: &mut Vec<VisibilityEdge>,
    ) {
        let d = to.center - from.center;
        let distance = d.length();
        let intersect_dist = distance * from.radius / (from.radius + to.radius);

        let base_angle = d.angle();
        let angle_flip = match from_dir {
            WindingDir::Clockwise => 1.0,
            WindingDir::Counterclockwise => -1.0,
        };
        let same_dir =
            base_angle + angle_flip * (PI / 2.0 + ((to.radius - from.radius) / distance).asin());
        let cross_dir = base_angle + angle_flip * (from.radius / intersect_dist).acos();

        if from.contains_angle(same_dir) && to.contains_angle(same_dir) {
            let same = Segment {
                from: from.center + Vec2f::new_angle(from.radius, same_dir),
                to: to.center + Vec2f::new_angle(to.radius, same_dir),
            };

            if self.is_segment_passable(&same, Some(from_id), Some(to_id)) {
                out.push(VisibilityEdge {
                    segment: same,
                    from_angle: same_dir,
                    to_angle: same_dir,
                    dest: ArcContext::new(to_id, from_dir),
                });
            }
        }

        if from.contains_angle(cross_dir) && to.contains_angle(cross_dir + PI) {
            let cross = Segment {
                from: from.center + Vec2f::new_angle(from.radius, cross_dir),
                to: to.center - Vec2f::new_angle(to.radius, cross_dir),
            };

            if self.is_segment_passable(&cross, Some(from_id), Some(to_id)) {
                out.push(VisibilityEdge {
                    segment: cross,
                    from_angle: cross_dir,
                    to_angle: cross_dir + PI,
                    dest: ArcContext::new(to_id, from_dir.opposite()),
                })
            }
        }
    }

    fn is_segment_passable(
        &self,
        seg: &Segment,
        ignore_a: Option<usize>,
        ignore_b: Option<usize>,
    ) -> bool {
        for (i, arc) in self.arcs.iter().enumerate() {
            if let Some(a) = ignore_a {
                if a == i {
                    continue;
                }
            }
            if let Some(b) = ignore_b {
                if b == i {
                    continue;
                }
            }

            let dist = arc.center.segment_dist_sq(seg.from, seg.to);
            if dist < arc.radius * arc.radius {
                return false;
            }
        }

        for segment in &self.segments {
            let d1 = segment.to - segment.from;
            let d2 = seg.to - seg.from;

            let vp = d1.x * d2.y - d2.x * d1.y;
            if vp.abs() < 0.001 {
                // Ignore collision if the segments are collinear
                // This is so a segment between two vertices of a polygon cannot
                // collide with its own edge
                continue;
            }

            let v = seg.from - segment.from;

            let k1 = (v.x * d2.y - v.y * d2.x) / vp;
            if k1 < 0.0 || k1 > 1.0 {
                continue;
            }

            let k2 = (v.x * d1.y - v.y * d1.x) / vp;
            if k2 < 0.0 || k2 > 1.0 {
                continue;
            }

            return false;
        }

        return true;
    }
}
