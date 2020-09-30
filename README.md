# Extended Kalman Filter
This is a project of the Udacity Self-Driving Car Nanodegree program. The aim of the project is to apply Extended Kalman Filter for fusion of data from LIDAR and Radar sensors of a self driving car using C++.

The project was created with the Udacity [Starter Code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

# Compiling and executing the project

These are the suggested steps:


### Docker setup in windows10 [link](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/forum_archive/brightwellt+Docker+Windows+Starter+Guide.pdf)

### Start the docker :
```
docker run -it --name SDC -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest
```

### Check docker :
```
..\ExtendedKF\src>docker ps

output :

CONTAINER ID        IMAGE                         COMMAND             CREATED             STATUS              PORTS                    NAMES
8719968d0b32        udacity/controls_kit:latest   "/bin/bash"         9 hours ago         Up 9 hours          0.0.0.0:4567->4567/tcp   SDC 
```

### open bash :
```
docker exec -it 8719968d0b32 bash

output:(docker bash) 

root@8719968d0b32:/work#

```

### Clone the repo :
```
root@8719968d0b32:/work# git clone https://github.com/YashCS90/KalmanFilter.git
```
### Content of this repo
- `scr` a directory with the project code:
  - `main.cpp` - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
  - `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
  - `kalman_filter.cpp`- defines the predict function, the update function for lidar, and the update function for radar
  - `tools.cpp` - a function to calculate RMSE and the Jacobian matrix
- `data`  a directory with two input files, provided by Udacity
- `Docs` a directory with files formats description


Create the build directory and run below command in sequence : 
```
root@8719968d0b32:/work/KalmanFilter# mkdir build
root@8719968d0b32:/work/KalmanFilter# cd build/
root@8719968d0b32:/work/KalmanFilter/build# cmake ..
-- Configuring done
-- Generating done
-- Build files have been written to: /work/KalmanFilter/build
root@8719968d0b32:/work/KalmanFilter/build# make
[100%] Built target ExtendedKF
root@8719968d0b32:/work/KalmanFilter/build# ./ExtendedKF
Listening to port 4567
Connected!!!

```



 

## Result


# [Rubric](https://review.udacity.com/#!/rubrics/748/view) points

## Compiling

### Your code should compile

The code compiles without errors.
```
root@8719968d0b32:/work/KalmanFilter# mkdir build
root@8719968d0b32:/work/KalmanFilter# cd build/
root@8719968d0b32:/work/KalmanFilter/build# cmake ..
-- Configuring done
-- Generating done
-- Build files have been written to: /work/KalmanFilter/build
root@8719968d0b32:/work/KalmanFilter/build# make
[100%] Built target ExtendedKF
root@8719968d0b32:/work/KalmanFilter/build# ./ExtendedKF
Listening to port 4567
Connected!!!
```

## Accuracy

### px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt which is the same data file the simulator uses for Dataset 1"


![input 1 results](images/Dataset1.png)
Accuracy - RMSE: [0.0975, 0.0857,  0.4527,  0.4416]
*Threshold*: RMSE <= [.11, .11, 0.52, 0.52]


![input 2 results](images/Dataset2.png)
Accuracy - RMSE: [0.0726, 0.0966, 0.4576, 0.4961]
*Threshold*: RMSE <= [.11, .11, 0.52, 0.52]

## Following the Correct Algorithm

### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

3 functions implemented :

```
void KalmanFilter::Predict() {
  /**
   * predict the state
   */
	x_ = F_ * x_;
	P_ = (F_ * P_ * F_.transpose()) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
	VectorXd y = z - (H_ * x_);
	MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
    MatrixXd K =  P_ * H_.transpose() * S.inverse();

    // new state
    x_ = x_ + (K * y);
    P_ = (MatrixXd::Identity(x_.size(), x_.size()) - (K * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
	// h(x)
	VectorXd h_x(3);
	h_x[0] = (double)sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
	h_x[1] = (double)atan2(x_[1], x_[0]);
	h_x[2] = (double)(x_[0]*x_[2] + x_[1]*x_[3]) / (sqrt(x_[0]*x_[0] + x_[1]*x_[1])); // / h_x[0]


	VectorXd y = z - h_x;
	while (y[1] >  M_PI) y[1] -= 2*M_PI;
	while (y[1] < -M_PI) y[1] += 2*M_PI;
	MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
    MatrixXd K =  P_ * H_.transpose() * S.inverse();

    // new state
    x_ = x_ + (K * y);
    P_ = (MatrixXd::Identity(x_.size(), x_.size()) - (K * H_)) * P_;
}
```


### Your Kalman Filter algorithm handles the first measurements appropriately.
```
 MeasurementPackage meas_package;
          std::istringstream iss(sensor_measurement);
          
          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro,theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }
```
### Your Kalman Filter algorithm first predicts then updates and can handle radar and lidar measurements.
```
  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  // measurement matrix
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  // measurement covariance matrix
	  ekf_.R_ =  R_radar_;
	  // update
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
	  // measurement matrix
	  ekf_.H_ = H_laser_;
	  // measurement covariance matrix
	  ekf_.R_ =  R_laser_;
	  // update
	  ekf_.Update(measurement_pack.raw_measurements_);
  }
```




