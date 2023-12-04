use bit_vec::BitVec;

use crate::vectors::Vec2i;

#[derive(Clone)]
pub struct Grid2D {
    passable: BitVec,
    size: Vec2i,
}

impl Grid2D {
    pub fn new(size: Vec2i) -> Self {
        Self {
            passable: BitVec::from_elem((size.x * size.y) as usize, true),
            size,
        }
    }

    fn cell_idx(&self, pos: &Vec2i) -> usize {
        (pos.x + pos.y * self.size.x) as usize
    }

    pub fn can_cell_pass(&self, x: i32, y: i32) -> bool {
        if x < 0 || y < 0 || x >= self.size.x || y >= self.size.y {
            return false;
        }

        self.passable
            .get(self.cell_idx(&Vec2i { x, y }))
            .unwrap_or(false)
    }

    pub fn can_point_pass(&self, pos: &Vec2i) -> bool {
        let mut pass = self.can_cell_pass(pos.x - 1, pos.y - 1);
        pass |= self.can_cell_pass(pos.x, pos.y - 1);
        pass |= self.can_cell_pass(pos.x - 1, pos.y);
        pass | self.can_cell_pass(pos.x, pos.y)
    }

    pub fn set_cell_passable(&mut self, pos: &Vec2i, passable: bool) {
        self.passable.set(self.cell_idx(pos), passable);
    }

    pub fn cell_size(&self) -> Vec2i {
        self.size
    }

    pub fn point_size(&self) -> Vec2i {
        Vec2i {
            x: self.size.x + 1,
            y: self.size.y + 1,
        }
    }

    pub fn heuristic(&self, point: &Vec2i, goal: &Vec2i) -> f64 {
        self.cost(point, goal)
    }

    pub fn cost(&self, current: &Vec2i, goal: &Vec2i) -> f64 {
        let dx = (current.x - goal.x) as f64;
        let dy = (current.y - goal.y) as f64;

        (dx * dx + dy * dy).sqrt()
    }

    pub fn get_all_neighbors(&self, current: &Vec2i, out: &mut Vec<Vec2i>) {
        out.clear();

        let Vec2i { x: w, y: h } = self.point_size();
        for x in (-1)..=1 {
            for y in (-1)..=1 {
                if x == 0 && y == 0 {
                    continue;
                }

                let px = current.x + x;
                let py = current.y + y;
                let p = Vec2i { x: px, y: py };

                if px >= 0 && px < w && py >= 0 && py < h {
                    out.push(p);
                }
            }
        }
    }

    pub fn get_visible_neighbors(&self, current: &Vec2i, out: &mut Vec<Vec2i>) {
        let ul = self.can_cell_pass(current.x - 1, current.y - 1);
        let ur = self.can_cell_pass(current.x, current.y - 1);
        let bl = self.can_cell_pass(current.x - 1, current.y);
        let br = self.can_cell_pass(current.x, current.y);

        out.clear();
        if ul {
            out.push(Vec2i {
                x: current.x - 1,
                y: current.y - 1,
            });
        }
        if ur {
            out.push(Vec2i {
                x: current.x + 1,
                y: current.y - 1,
            });
        }
        if bl {
            out.push(Vec2i {
                x: current.x - 1,
                y: current.y + 1,
            });
        }
        if br {
            out.push(Vec2i {
                x: current.x + 1,
                y: current.y + 1,
            });
        }

        if ul || ur {
            out.push(Vec2i {
                x: current.x,
                y: current.y - 1,
            });
        }
        if bl || br {
            out.push(Vec2i {
                x: current.x,
                y: current.y + 1,
            });
        }
        if ul || bl {
            out.push(Vec2i {
                x: current.x - 1,
                y: current.y,
            });
        }
        if ur || br {
            out.push(Vec2i {
                x: current.x + 1,
                y: current.y,
            });
        }
    }

    // This implementation was taken from somewhere
    // TODO: Figure out where so credit can be properly given
    pub fn line_of_sight(&self, s: &Vec2i, sp: &Vec2i) -> bool {
        let mut x0 = s.x;
        let mut y0 = s.y;
        let x1 = sp.x;
        let y1 = sp.y;
        let mut delta_y = y1 - y0;
        let mut delta_x = x1 - x0;
        let mut f = 0;

        let step_y;
        let step_x;

        if delta_y < 0 {
            delta_y = -delta_y;
            step_y = -1;
        } else {
            step_y = 1;
        }

        if delta_x < 0 {
            delta_x = -delta_x;
            step_x = -1;
        } else {
            step_x = 1;
        }

        if delta_x >= delta_y {
            while x0 != x1 {
                f = f + delta_y;
                if f >= delta_x {
                    if !self.can_cell_pass(x0 + ((step_x - 1) / 2), y0 + ((step_y - 1) / 2)) {
                        return false;
                    }
                    y0 = y0 + step_y;
                    f = f - delta_x;
                }
                if f != 0 && !self.can_cell_pass(x0 + ((step_x - 1) / 2), y0 + ((step_y - 1) / 2)) {
                    return false;
                }
                if delta_y == 0
                    && !self.can_cell_pass(x0 + ((step_x - 1) / 2), y0)
                    && !self.can_cell_pass(x0 + ((step_x - 1) / 2), y0 - 1)
                {
                    return false;
                }
                x0 = x0 + step_x;
            }
        } else {
            while y0 != y1 {
                f = f + delta_x;
                if f >= delta_y {
                    if !self.can_cell_pass(x0 + ((step_x - 1) / 2), y0 + ((step_y - 1) / 2)) {
                        return false;
                    }
                    x0 = x0 + step_x;
                    f = f - delta_y;
                }
                if f != 0 && !self.can_cell_pass(x0 + ((step_x - 1) / 2), y0 + ((step_y - 1) / 2)) {
                    return false;
                }
                if delta_x == 0
                    && !self.can_cell_pass(x0, y0 + ((step_y - 1) / 2))
                    && !self.can_cell_pass(x0 - 1, y0 + ((step_y - 1) / 2))
                {
                    return false;
                }
                y0 = y0 + step_y;
            }
        }

        return true;
    }

    pub fn get_as_java_bitset(&self) -> Vec<u64> {
        let bit_count = (self.size.x * self.size.y) as usize;
        let mut data = vec![0u64; (bit_count + 63) / 64];

        for i in 0..bit_count {
            let word_idx = i / 64;
            let bit_idx = i % 64;

            if self.passable.get(i).unwrap_or(false) {
                data[word_idx] = data[word_idx] | (1u64 << bit_idx);
            }
        }

        data
    }
}
