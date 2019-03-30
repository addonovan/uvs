use std::sync::{Arc, Mutex};

mod messages;
pub use self::messages::*;

mod environment;
use environment::Environment;
use rosrust::Message;

struct GlobalData<T : Message> {
    env: Environment,
    publisher: rosrust::api::raii::Publisher<T>,
}

fn main() {
    rosrust::init("uvs_reactive");

    let global = GlobalData {
        env: Environment::new(),
        publisher: rosrust::publish("theta_deflection").unwrap(),
    };
    let global = Arc::new(Mutex::new(global));

    let data = global.clone();
    rosrust::subscribe("sonar", move |message: std_msgs::UInt16| {
        let mut data = data.lock().unwrap();

        data.env.sonar(message);
        let deflection = data.env.deflection();

        let mut message = std_msgs::Float64::default();
        message.data = deflection.inner();
        data.publisher.send(message).unwrap();
    }).unwrap();

    let data = global.clone();
    rosrust::subscribe("lidar", move |message: sensor_msgs::LaserScan| {
        let mut data = data.lock().unwrap();
        data.env.lidar(message);
    }).unwrap();

    rosrust::spin();
}
