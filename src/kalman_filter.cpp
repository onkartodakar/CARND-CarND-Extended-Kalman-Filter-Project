#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

MatrixXd ConvertToRadarMeas(const VectorXd& x_state);

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //std::cout<< "\n In Predict";
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  //cout<<"\n In update Laser";
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //cout<<"\n In update Radar.";
  VectorXd z_pred = ConvertToRadarMeas(x_);
  VectorXd y = z - z_pred;

  while (y(1) > ( M_PI)) {
          y(1) = y(1)-(2*M_PI);
      }
  
  while (y(1) < -(M_PI)) {
          y(1) = y(1)+ (2*M_PI);
      }


  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

MatrixXd ConvertToRadarMeas(const VectorXd& x_state) {

  //cout<<"\n In ConvertToRadarMeas";
  VectorXd z_radar;
  z_radar = VectorXd(3);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

 
  z_radar(0) = sqrt(pow(px,2)+pow(py,2));
  z_radar(1) = atan2(py,px);

  // Normalize theta to be in -pi to pi range
  while (z_radar(1) > ( M_PI)) {
          z_radar(1) = z_radar(1)-(2*M_PI);
      }
  
  while (z_radar(1) < -(M_PI)) {
          z_radar(1) = z_radar(1)+ (2*M_PI);
      }

   //check division by zero    
  if (z_radar(0)<.0001){
      z_radar(2) = (px*vx+py*vy)/0.0001;
      }
  else{
      z_radar(2) = (px*vx+py*vy)/z_radar(0);
  }

  return z_radar;
}

