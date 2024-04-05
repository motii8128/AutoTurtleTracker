use async_std;
use safe_drive::{
    context::Context,
    error::DynError,
    msg::common_interfaces::{geometry_msgs, nav_msgs}
};

use velocity_controller::velocity_controller;

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("VelocityController", None, Default::default())?;

    let sub_target_odom = node.create_subscriber::<nav_msgs::msg::Odometry>("/odom/target", None)?;
    let sub_real_odom = node.create_subscriber::<nav_msgs::msg::Odometry>("/odom/real", None)?;
    let pub_cmd = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;

    let task = async_std::task::spawn(velocity_controller(sub_target_odom, sub_real_odom, pub_cmd, 10));

    task.await?;

    Ok(())
}
