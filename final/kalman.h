#include "matrix.h"
#include <math.h>

class Kalman {
    public:
        Kalman();
        ~Kalman();
        Kalman(const Matrix&);
        
        void init(Matrix, Matrix);
        
        void setAll();
        
        void setA();
        void makeA();
        Matrix getA();
        
        void setH();
        void makeH();
        Matrix getH();
        
        void setQ();
        void makeQ();
        Matrix getQ();
        
        void setR();
        void makeR();
        Matrix getR();
        
        Matrix getX();
        
        Matrix step(Matrix, double);
        
        void update_vel(double vel);
        void update_heading(double);
        
    private:
        Matrix X, P, A, H, Q, R;
        double sample_time, velocity, heading;
        
};
