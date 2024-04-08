use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::common_interfaces::geometry_msgs,
};
use async_std;
use dualshock_driver::{DualShock4Driver, BLE, SERIAL};
use ros2_rust_util::{get_bool_parameter, get_str_parameter};

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("JoyTwistRust", None, Default::default())?;
    let log = Logger::new(node.get_name().unwrap().as_str());

    let publisher = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;

    let mode_name = get_str_parameter(node.get_name().unwrap().as_str(), "mode", "ble");
    let enable_debug = get_bool_parameter(node.get_name().unwrap().as_str(), "enable_debug", false);

    let mode = name_to_mode(&mode_name);
    let mut driver = DualShock4Driver::new(mode).unwrap();

    pr_info!(log, "Start {}", node.get_name().unwrap());
    loop {
        let con = driver.read().await.unwrap();

        let mut msg = geometry_msgs::msg::Twist::new().unwrap();

        msg.linear.x = con.sticks.left_x as f64;
        msg.linear.y = con.sticks.left_y as f64;

        msg.angular.z = con.sticks.right_x as f64;

        let _ = publisher.send(&msg).unwrap();
        if enable_debug
        {
            pr_info!(log, "Send linear.x:{}, linear.y:{}, angular.z:{}", msg.linear.x, msg.linear.y, msg.angular.z);
        }
    }
}

fn name_to_mode(name:&str)->u8
{
    if name == "ble"
    {
        BLE
    }
    else if name == "serial"
    {
        SERIAL
    }
    else {
        0
    }
}