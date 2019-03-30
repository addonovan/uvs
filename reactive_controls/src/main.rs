mod messages;
pub use self::messages::*;

mod environment;
use environment::Environment;

fn main() {
    rosrust::init("uvs_reactive");

    println!("Hello, world!");
}
