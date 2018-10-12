#include "kalman.h"


Kalman::Kalman ()
{
    A = Matrix(3, 3);
    X = Matrix(3, 1);
    Q = Matrix(3, 3);
    R = Matrix(3, 3);
    P = Matrix(3, 3);
    H = Matrix(3, 3);
    sample_time = 1.;
}

Kalman::~Kalman ()
{

}

void Kalman::init(Matrix x, Matrix p){
    X = x;
    P = p;
    setAll();
}

void Kalman::setAll(){
    setA();
    setR();
    setQ();
    setH();
}

void Kalman::update_vel(double vel){
    velocity = vel;
}

void Kalman::update_heading(double h){
    heading = h;
}

void Kalman::setA ()
{
  A (0, 0) = 1;
  A (0, 1) = 0;
  //A (0, 2) = 1;
  
  A (1, 0) = 0;
  A (1, 1) = 1;
  //A (1, 2) = 1;
  
  A (2, 0) = 0;
  A (2, 1) = 0;
  A (2, 2) = 1;
}

void Kalman::makeA(){
    A(0, 2) = -velocity*sample_time*sin(heading);
    A(1, 2) = velocity*sample_time*cos(heading);
}

Matrix Kalman::getA(){
    return A;
}

void Kalman::setH ()
{
  H(0, 0) = 1;
  H(0, 1) = 0;
  H(0, 2) = 0;
  
  H(1, 0) = 0;
  H(1, 1) = 1;
  H(1, 2) = 0;
  
  H(2, 0) = 0;
  H(2, 1) = 0;
  H(2, 2) = 1;
}

void Kalman::makeH(){
    
}

Matrix Kalman::getH(){
    return H;
}

void Kalman::setQ ()
{
  Q(0, 0) = 0.05;
  Q(0, 1) = 0;
  Q(0, 2) = 0;
  Q(1, 0) = 0;
  Q(1, 1) = 0.05;
  Q(1, 2) = 0;
  Q(2, 0) = 0;
  Q(2, 1) = 0;
  Q(2, 2) = 0.0001;
}

Matrix Kalman::getQ(){
    return Q;
}

void Kalman::setR ()
{
  R(0, 0) = 0.05;
  R(0, 1) = 0;
  R(0, 2) = 0;
  R(1, 0) = 0;
  R(1, 1) = 0.05;
  R(1, 2) = 0;
  R(2, 0) = 0;
  R(2, 1) = 0;
  R(2, 2) = 0.00008;
}

Matrix Kalman::getR(){
    return R;
}

Matrix Kalman::getX(){
    return X;
}

Matrix Kalman::step(Matrix Z, double vel){
    update_vel(vel);
    update_heading(Z(2, 0));
    makeA();
    
    Matrix X_hat = A*X;
    Matrix P_hat = A*P*(A.transpose()) + Q;
    Matrix HPhatHTPLUSR = H*P_hat*(H.transpose()) + R;
    Matrix K = P_hat*(H.transpose())*HPhatHTPLUSR.inverse();
    
    X = X_hat + K*(Z - H*X_hat);
    P = P_hat - K*H*P_hat;
    return X;
}
