# include <cmath>

class VehicleDynamics
{
    public:
        VehicleDynamics();
        void update_simulation();
    
    private:
        void calculate_dynamics();
        void integrate_dynamics();
};