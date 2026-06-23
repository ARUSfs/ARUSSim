#include <cmath>
#include <algorithm>
#include <iostream>
#include "arussim/csv_generator.hpp"
#include <fstream>
class VehicleDynamics
{
    public:
        VehicleDynamics();
        void update_simulation(double input_delta, std::vector<double> input_torque, double dt);
        void set_parameters(std::map<std::string, double> &params);

        double x_;
        double y_;
        double yaw_;
        double vx_;
        double vy_;
        double r_;
        double ax_;
        double ay_;
        double delta_;
        double delta_fl_;
        double delta_fr_;
        double delta_v_;

        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } wheel_speed_;
        
        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } torque_cmd_;

        struct {
            double alpha_fl_ = 0;
            double alpha_fr_ = 0;
            double alpha_rl_ = 0;
            double alpha_rr_ = 0;

            double lambda_fl_ = 0;
            double lambda_fr_ = 0;
            double lambda_rl_ = 0;
            double lambda_rr_ = 0;
        } tire_slip_;
        
        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } tire_loads_;
        
        double input_delta_;
        double input_acc_;
        double dt_;

        void write_csv_row();
        void set_csv_generator(std::shared_ptr<CSVGenerator> csvGen) {
            csv_generator_vehicle_dynamics_ = csvGen;
        }
    
    private:
        
        double kMass;
        double kNsMassF;
        double kNsMassR;
        double kSMass;
        double kIzz;
        double kHCoGNsMassF;
        double kHCoGNsMassR;
        double kHCoGsMass;
        double kPres;

        double kMassDistributionRear;
        double kSMassF;
        double kSMassR;

        double kWheelBase;
        double kTrackWidth;
        double kLf;
        double kLr;

        double kHCog;
        double kHCogNsF;
        double kHCogNsR;
        double kHRollCenterF;
        double kHRollCenterR;
        double kHRollAxis;

        double kSpringStiffnessF;
        double kSpringStiffnessR;
        double kMotionRatioF;
        double kMotionRatioR;        
        double kWheelRateF;
        double kWheelRateR;
        double kRollStiffnessF;
        double kRollStiffnessR;
        double kRollStiffness;

        double kAckermann1;
        double kAckermann2;

        double kTireDynRadius;
        double kTireInertia_F;
        double kTireInertia_R;

        double kGearRatio;

    struct {
        // PARAMETROS GENERALES Y ESCALAS
        double LONGVL, NOMPRES, FNOMIN, UNLOADED_RADIUS;
        double LFZO, LMUX, LMUY, LKX, LKY, LKYC, LHY, LTR, LRES, LKZC, LXAL, LYKA, LVYKA, LS;

        // FUERZA LONGITUDINAL PURA (Fx0)
        double LCX, LEX, LHX, LVX;
        double PCX1, PDX1, PDX2, PDX3;
        double PEX1, PEX2, PEX3, PEX4;
        double PKX1, PKX2, PKX3;
        double PHX1, PHX2;
        double PVX1, PVX2;
        double PPX1, PPX2, PPX3, PPX4;

        // FUERZA LATERAL PURA (Fy0)
        double LCY, LEY, LVY;
        double PCY1, PDY1, PDY2, PDY3;
        double PEY1, PEY2, PEY3, PEY4, PEY5;
        double PKY1, PKY2, PKY3, PKY4, PKY5;
        double PHY1, PHY2, PHY3;
        double PVY1, PVY2, PVY3, PVY4;
        double PPY1, PPY2, PPY3, PPY4;

        // MOMENTO DE AUTOALINEAMIENTO PURO (Mz0)
        double QBZ1, QBZ2, QBZ3, QBZ4, QBZ5, QBZ9, QBZ10;
        double QCZ1;
        double QDZ1, QDZ2, QDZ3, QDZ4, QDZ6, QDZ7, QDZ8, QDZ9, QDZ10, QDZ11;
        double QEZ1, QEZ2, QEZ3, QEZ4, QEZ5;
        double QHZ1, QHZ2, QHZ3, QHZ4;
        double PPZ1, PPZ2;
        double SSZ1, SSZ2, SSZ3, SSZ4;

        // COEFICIENTES DE TRACCIÓN / GIRO COMBINADO (Gxa y Gyk)
        double RBX1, RBX2, RBX3, RCX1, REX1, REX2, RHX1;
        double RBY1, RBY2, RBY3, RBY4, RCY1, REY1, REY2, RHY1, RHY2, RVY1, RVY2, RVY3, RVY4, RVY5, RVY6;

    } pac_param_;

        double kRollingResistance;
        double kCDA;
        double kCLA;
        double kCOPx; //longitudinal distribution (rear)
        double kCOPy;
        double kAirDensity;

        double x_dot_{0.0}, y_dot_{0.0}, vx_dot_{0.0}, vy_dot_{0.0}, r_dot_{0.0};
        double w_fl_dot_{0.0}, w_fr_dot_{0.0}, w_rl_dot_{0.0}, w_rr_dot_{0.0};
        double delta_dot_{0.0}, delta_v_dot_{0.0};

        double kG;

        double kStaticLoadFront;
        double kStaticLoadRear;

        struct Tire_force {
            double fy;
            double fx;
            double mz;
        };

        Tire_force force;

        // Steering dynamics
        double kCoefDelta = 306.3;
        double kCoefV = 25.69;
        double kCoefInput = 307;
        double kSteeringAMax = 3.0;
        double kSteeringVMax = 2.3;

        void calculate_dynamics();
        void integrate_dynamics();

        double calculate_fx(Tire_force force_fl, Tire_force force_fr, Tire_force force_rl, Tire_force force_rr);

        void calculate_tire_loads();
        void calculate_ackermann();
        void calculate_tire_slip();
        Tire_force calculate_tire_forces(double slip_angle, double slip_ratio, double tire_load, double camber, double pressure);
        void kinematic_correction();

        std::shared_ptr<CSVGenerator> csv_generator_vehicle_dynamics_;
};