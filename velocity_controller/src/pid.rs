pub struct PID
{
    pub p_gain:f64,
    pub i_gain:f64,
    pub d_gain:f64,

    pub integral:f64,
    pub prev_proportional:f64
}

impl PID {
    pub fn new(p:f64, i:f64, d:f64)->PID
    {
        PID{p_gain:p, i_gain:i, d_gain:d, integral:0.0, prev_proportional:0.0}
    }

    pub fn calc(&mut self,target:f64, history:f64, delta_time:f64)->f64
    {
        let proportional = target - history;

        self.integral += proportional * delta_time;

        let differential = (proportional - self.prev_proportional) / delta_time;

        self.prev_proportional = proportional;

        proportional * self.p_gain + self.integral * self.i_gain + differential * self.d_gain
    }
}