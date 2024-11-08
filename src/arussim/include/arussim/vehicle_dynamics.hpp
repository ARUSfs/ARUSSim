#include <cmath>
#include <algorithm>

class VehicleDynamics
{
    public:
        VehicleDynamics();
        void update_simulation(double input_delta, double input_acc, double dt);

        double x_;
        double y_;
        double yaw_;
        double vx_;
        double vy_;
        double r_;
        double input_delta_;
        double input_acc_;
        double dt_;
    
    private:

        double kMass = 270.0;
        double kWheelBase = 1.535;

        double kMinFx = -3500;
        double kMaxFx = 2000;

        double kRollingResistance = 100;
        double kCDA = 1;
        double kAirDensity = 1.1;

        double x_dot_{0.0}, y_dot_{0.0}, yaw_dot_{0.0}, vx_dot_{0.0};

        void calculate_dynamics();
        void integrate_dynamics();
        double calculate_fx();
};