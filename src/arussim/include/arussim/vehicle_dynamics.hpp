# include <cmath>

class VehicleDynamics
{
    public:
        VehicleDynamics();
        void update_simulation(double& x, double& y, double& yaw, double& vx, double input_delta, double input_acc, double dt);
    
    private:
        double kWheelBase = 1.533;

        double x_dot_{0.0}, y_dot_{0.0}, yaw_dot_{0.0};

        void calculate_dynamics(double vx, double input_delta);
        void integrate_dynamics(double& x, double& y, double& yaw, double& vx, double input_acc, double dt);
};