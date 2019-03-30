pub struct Radian {
    value: f64,
}

impl Radian {
    pub fn new(value: f64) -> Self {
        Radian { value }
    }

    pub fn inner(&self) -> f64 {
        self.value
    }
}

pub struct Centimeter {
    value: i32,
}

impl Centimeter {
    pub fn new(value: i32) -> Self {
        Centimeter { value }
    }

    pub fn inner(&self) -> i32 {
        self.value
    }
}
