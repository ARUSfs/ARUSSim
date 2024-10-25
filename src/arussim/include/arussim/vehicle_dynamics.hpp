#include <cmath>
#include <algorithm>

class VehicleDynamics
{
    public:
        VehicleDynamics();
        void update_simulation(double& x, double& y, double& yaw, double& vx, 
                                double input_delta, double input_acc, double dt);
    
    private:
        double kWheelBase = 1.533;
        double kMass = 230;

        double kMinFx = -3500;
        double kMaxFx = 2000;

        double kRollingResistance = 100;
        double kCDA = 1;
        double kAirDensity = 1.1;

        double x_dot_{0.0}, y_dot_{0.0}, yaw_dot_{0.0}, vx_dot_{0.0};

        void calculate_dynamics(double vx, double yaw, double input_delta, double input_acc);
        void integrate_dynamics(double& x, double& y, double& yaw, double& vx, double dt);
        double calculate_fx(double vx, double input_acc);
};