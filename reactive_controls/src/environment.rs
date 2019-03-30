use utils::unit::Radian;

use crate::std_msgs::UInt16;
use crate::sensor_msgs::LaserScan;

const SONAR_THRESHOLD: i32 = 200; // [cm]
const LIDAR_THRESHOLD: i32 = 200; // [cm]

pub struct Environment {
    sonar: i32,
    lidar: Vec<(Radian, f64)>,
}

impl Environment {
    pub fn new() -> Self {
        Environment {
            sonar: SONAR_THRESHOLD + 1,
            lidar: Vec::new(),
        }
    }

    pub fn sonar(&mut self, reading: UInt16) {
        self.sonar = reading.data as i32;
    }

    pub fn lidar(&mut self, reading: LaserScan) {
        let angle_min: f64 = reading.angle_min as f64;
        let angle_step: f64 = reading.angle_increment as f64;
        let range_min: f32 = reading.range_min;
        let range_max: f32 = reading.range_max;

        // first, add type annotations to the values, just so
        // we can get type help
        let ranges: Vec<f32> = reading.ranges;
        self.lidar = ranges.into_iter()
            .enumerate()
            .filter(|(_, it)| *it >= range_min && *it <= range_max)
            .map(|(i, it)| {
                let i: f64 = i as f64;
                let angle = Radian::new(angle_min + (i * angle_step));
                let range = it as f64;
                (angle, range)
            })
            .collect();
    }

    pub fn deflection(&self) -> Radian {
        Radian::new(0.)
    }
}
