pub mod pid;

use safe_drive::{
    topic::{subscriber::Subscriber, publisher::Publisher},
    msg::common_interfaces::{nav_msgs, geometry_msgs},
    logger::Logger,
    pr_info,
    error::DynError
};

use pid::PID;

pub async fn velocity_controller(
    mut sub_target_odom:Subscriber<nav_msgs::msg::Odometry>,
    mut sub_history_odom:Subscriber<nav_msgs::msg::Odometry>,
    pub_cmd:Publisher<geometry_msgs::msg::Twist>,
    cmd_freqency:u64
)->Result<(), DynError>
{
    let log = Logger::new("VelocityController");

    let mut x_pid = PID::new(0.1, 0.1, 0.1);
    let mut y_pid = PID::new(0.1, 0.1, 0.1);
    let mut rotation_pid = PID::new(0.1, 0.1, 0.1);

    let mut prev_vel_x = 0.0;
    let mut prev_vel_y = 0.0;

    let delta_sec = 1.0 / cmd_freqency as f64 * 10e-4;

    pr_info!(log, "Start VelocityController on {}Hz", cmd_freqency);
    loop {
        let mut result_cmd = geometry_msgs::msg::Twist::new().unwrap();
        let target_odom = sub_target_odom.recv().await.unwrap();
        let real_odom = sub_history_odom.recv().await.unwrap();

        let target_vel_x = (target_odom.pose.pose.position.x - real_odom.pose.pose.position.x) / delta_sec;
        let target_vel_y = (target_odom.pose.pose.position.y - real_odom.pose.pose.position.y) / delta_sec;

        result_cmd.linear.x = x_pid.calc(target_vel_x, prev_vel_x, delta_sec);
        result_cmd.linear.y = y_pid.calc(target_vel_y, prev_vel_y, delta_sec);

        let _ = pub_cmd.send(&result_cmd);

        prev_vel_x = target_vel_x;
        prev_vel_y = target_vel_y;

        std::thread::sleep(std::time::Duration::from_millis(delta_sec as u64))
    }
}