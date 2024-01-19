use std::f64::consts::PI;

use lerp::Lerp;

use crate::{
    geom::{PathArc, WindingDir},
    math::{self, Vec2f},
};

pub fn to_bezier(path: &Vec<PathArc>, start: Vec2f, goal: Vec2f) -> Vec<Vec2f> {
    // if path.is_empty() {
    //     let cp1 = start.lerp(goal, 1.0 / 3.0);
    //     let cp2 = start.lerp(goal, 2.0 / 3.0);

    //     return vec![start, cp1, cp2, goal];
    // }

    let mut bezier_pts = Vec::new();

    let mut last_pt = start;
    for arc in path {
        let angle1 = math::wrap_angle(arc.incoming_angle);
        let angle2 = math::wrap_angle(arc.outgoing_angle);
        let diff = math::floor_mod(
            match arc.direction {
                WindingDir::Counterclockwise => angle2 - angle1,
                WindingDir::Clockwise => angle1 - angle2,
            },
            PI * 2.0,
        );

        let in_pt = arc.center + Vec2f::new_angle(arc.radius, angle1);
        let out_pt = arc.center + Vec2f::new_angle(arc.radius, angle2);

        let l = 4.0 / 3.0 * (diff / 4.0).tan() * arc.radius;

        // FIXME: Split the arc if it's over 180 degrees, since the approximation becomes significantly worse there
        //  Not fixing this year, field has no geometry that can cause this problem

        bezier_pts.push(last_pt);
        bezier_pts.push(last_pt.lerp(in_pt, 1.0 / 3.0));
        bezier_pts.push(last_pt.lerp(in_pt, 2.0 / 3.0));
        bezier_pts.push(in_pt);
        bezier_pts.push(in_pt + (in_pt - last_pt).norm() * l);
        bezier_pts.push(
            out_pt
                + Vec2f::new_angle(
                    l,
                    angle2
                        + match arc.direction {
                            WindingDir::Clockwise => PI / 2.0,
                            WindingDir::Counterclockwise => -PI / 2.0,
                        },
                ),
        );

        last_pt = out_pt;
    }

    bezier_pts.push(last_pt);
    bezier_pts.push(last_pt.lerp(goal, 1.0 / 3.0));
    bezier_pts.push(last_pt.lerp(goal, 2.0 / 3.0));
    bezier_pts.push(goal);

    bezier_pts
}
