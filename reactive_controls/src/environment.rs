use utils::unit::Radian;

use crate::std_msgs::UInt16;
use crate::sensor_msgs::LaserScan;
use std::cmp::Ordering;

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
        let sonar_deflection = self.sonar_deflection();
        if sonar_deflection.abs() > 0.01 {
            return Radian::new(sonar_deflection);
        }

        let lidar_deflection = self.lidar_deflection();
        if lidar_deflection.abs() > 0.01 {
            return Radian::new(lidar_deflection);
        }

        return Radian::new(0.0);
    }

    fn lidar_deflection(&self) -> f64 {
        use std::f64::consts::PI;

        self.lidar.iter()
            .min_by(|(_, a), (_, b)| {
                // let's face it, floats'll never really be equal
                if a < b {
                    Ordering::Less
                } else {
                    Ordering::Greater
                }
            })
            .map(|(angle, reading)| {
                let thresh = LIDAR_THRESHOLD as f64;
                let scale = (thresh - reading) / thresh;
                let angle = if angle.inner() > 0.0 {
                    -3.0 * PI / 4.0
                } else {
                    3.0 * PI / 4.0
                };

                scale * angle
            })
            .unwrap_or(0.0)
    }

    fn sonar_deflection(&self) -> f64 {
        use std::f64::consts::PI;

        if self.sonar >= SONAR_THRESHOLD {
            0.0
        } else {
            let scale = (SONAR_THRESHOLD - self.sonar) as f64 / (SONAR_THRESHOLD as f64);
            let angle = 3.0 * PI / 4.0;

            scale * angle
        }
    }
}
