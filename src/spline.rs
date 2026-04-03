type Fx = i32;
const FP_SHIFT: i32 = 16;
const FP_ONE: Fx = 1 << FP_SHIFT;

#[inline(always)]
fn to_fx(x: i32) -> Fx {
    x << FP_SHIFT
}

#[inline(always)]
fn from_fx(x: Fx) -> i32 {
    (x + (FP_ONE >> 1)) >> FP_SHIFT
}

#[inline(always)]
fn mul_fx(a: Fx, b: Fx) -> Fx {
    ((a as i64 * b as i64) >> FP_SHIFT) as Fx
}

pub fn compute_steps(p0: (i32, i32), p1: (i32, i32), p2: (i32, i32)) -> u32 {
    let dx = (p2.0 - p1.0).abs();
    let dy = (p2.1 - p1.1).abs();
    let length = dx.max(dy);

    let curve_x = (p0.0 - 2 * p1.0 + p2.0).abs();
    let curve_y = (p0.1 - 2 * p1.1 + p2.1).abs();
    let curvature = curve_x.max(curve_y);

    let steps = length + (curvature >> 1);

    steps.clamp(1, 64) as u32
}

pub struct SplineRasterIter<'a> {
    spline: SplineIter<'a>,

    // Bresenham state
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,

    dx: i32,
    dy: i32,
    sx: i32,
    sy: i32,
    err: i32,

    active: bool,
}

impl<'a> SplineRasterIter<'a> {
    pub fn new(points: &'a [(i32, i32)], steps_per_segment: u32) -> Self {
        let mut spline = SplineIter::new(points, steps_per_segment);

        let first = spline.next().unwrap_or((0, 0));

        Self {
            spline,
            x0: first.0,
            y0: first.1,
            x1: first.0,
            y1: first.1,
            dx: 0,
            dy: 0,
            sx: 0,
            sy: 0,
            err: 0,
            active: false,
        }
    }

    fn start_line(&mut self, x1: i32, y1: i32) {
        self.x1 = x1;
        self.y1 = y1;

        self.dx = (self.x1 - self.x0).abs();
        self.sx = if self.x0 < self.x1 { 1 } else { -1 };

        self.dy = -(self.y1 - self.y0).abs();
        self.sy = if self.y0 < self.y1 { 1 } else { -1 };

        self.err = self.dx + self.dy;
        self.active = true;
    }
}

impl<'a> Iterator for SplineRasterIter<'a> {
    type Item = (i32, i32);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            // If currently rasterizing a line
            if self.active {
                let pt = (self.x0, self.y0);

                if self.x0 == self.x1 && self.y0 == self.y1 {
                    self.active = false;
                    return Some(pt);
                }

                let e2 = 2 * self.err;

                if e2 >= self.dy {
                    self.err += self.dy;
                    self.x0 += self.sx;
                }

                if e2 <= self.dx {
                    self.err += self.dx;
                    self.y0 += self.sy;
                }

                return Some(pt);
            }

            // Otherwise fetch next spline point
            let next_pt = self.spline.next()?;

            if next_pt != (self.x0, self.y0) {
                self.start_line(next_pt.0, next_pt.1);
                continue;
            }

            // Skip duplicates
        }
    }
}

pub struct SplineIter<'a> {
    points: &'a [(i32, i32)],
    segment: usize,
    steps: u32,
    step_idx: i32,

    x: Fx,
    y: Fx,

    dx: Fx,
    dy: Fx,

    ddx: Fx,
    ddy: Fx,

    dddx: Fx,
    dddy: Fx,
}

impl<'a> SplineIter<'a> {
    pub fn new(points: &'a [(i32, i32)], steps_per_segment: u32) -> Self {
        let mut s = Self {
            points,
            segment: 0,
            steps: steps_per_segment,
            step_idx: 0,
            x: 0,
            y: 0,
            dx: 0,
            dy: 0,
            ddx: 0,
            ddy: 0,
            dddx: 0,
            dddy: 0,
        };

        if points.len() >= 2 {
            s.setup_segment();
        }

        s
    }

    fn setup_segment(&mut self) {
        let last = self.points.len() - 1;
        if self.segment >= last {
            return;
        }

        let p1 = self.points[self.segment];
        let p2 = self.points[self.segment + 1];

        let p0 = if self.segment > 0 {
            self.points[self.segment - 1]
        } else {
            p1
        };

        let p3 = if self.segment + 2 <= last {
            self.points[self.segment + 2]
        } else {
            p2
        };

        let (x0, y0) = (to_fx(p0.0), to_fx(p0.1));
        let (x1, y1) = (to_fx(p1.0), to_fx(p1.1));
        let (x2, y2) = (to_fx(p2.0), to_fx(p2.1));
        let (x3, y3) = (to_fx(p3.0), to_fx(p3.1));

        // Catmull-Rom coefficients in fixed point
        let ax = (-x0 + 3 * x1 - 3 * x2 + x3) >> 1;
        let bx = (2 * x0 - 5 * x1 + 4 * x2 - x3) >> 1;
        let cx = (-x0 + x2) >> 1;
        let dx = x1;

        let ay = (-y0 + 3 * y1 - 3 * y2 + y3) >> 1;
        let by = (2 * y0 - 5 * y1 + 4 * y2 - y3) >> 1;
        let cy = (-y0 + y2) >> 1;
        let dy = y1;

        // dt in fixed point
        let n = self.steps as Fx;
        let dt = (FP_ONE / n) as Fx;
        let dt2 = mul_fx(dt, dt);
        let dt3 = mul_fx(dt2, dt);

        self.x = dx;
        self.y = dy;

        // Forward differences
        self.dx = mul_fx(ax, dt3) + mul_fx(bx, dt2) + mul_fx(cx, dt);
        self.dy = mul_fx(ay, dt3) + mul_fx(by, dt2) + mul_fx(cy, dt);

        self.ddx = mul_fx(6 * ax, dt3) + mul_fx(2 * bx, dt2);
        self.ddy = mul_fx(6 * ay, dt3) + mul_fx(2 * by, dt2);

        self.dddx = mul_fx(6 * ax, dt3);
        self.dddy = mul_fx(6 * ay, dt3);

        self.step_idx = 0;
    }
}

impl<'a> Iterator for SplineIter<'a> {
    type Item = (i32, i32);

    fn next(&mut self) -> Option<Self::Item> {
        if self.points.len() < 2 || self.segment >= self.points.len() - 1 {
            return None;
        }

        let out = (from_fx(self.x), from_fx(self.y));

        // additions only (fast path)
        self.x += self.dx;
        self.y += self.dy;

        self.dx += self.ddx;
        self.dy += self.ddy;

        self.ddx += self.dddx;
        self.ddy += self.dddy;

        self.step_idx += 1;

        if self.step_idx >= self.steps as i32 {
            self.segment += 1;
            if self.segment < self.points.len() - 1 {
                self.setup_segment();
            }
        }

        Some(out)
    }
}
