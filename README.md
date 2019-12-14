# CarND-Project-10-MPC-Model-Predictive-Control
Udacity Self-Driving Car Engineer Nanodegree: Project 10 - Model Predictive Control

## main.cpp

### Input

```cpp
double px = j[1]["x"];
double py = j[1]["y"];
double psi = j[1]["psi"];
double v = j[1]["speed"];
double delta = j[1]["steering_angle"];
double a = j[1]["throttle"];
```

### Transform the points to the vehicle's orientation

```cpp
for (int i = 0; i < ptsx.size(); i++) {
  double x = ptsx[i] - px;
  double y = ptsy[i] - py;
  ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
  ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
}
```

### Fits a 3rd-order polynomial to the vehicle's x and y coordinates

```cpp
auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
```

### Calculates the cross track error

Because points were transformed to vehicle coordinates, x & y equal 0 below.

```cpp
double cte = polyeval(coeffs, 0);
```

### Calculate the orientation error
    
Derivative of the polyfit goes in atan() below.

Because x = 0 in the vehicle coordinates, the higher orders are zero, leaves only coeffs[1].

```cpp
double epsi = -atan(coeffs[1]);
```

### Predict state after latency

```cpp
// Latency for predicting time at actuation
const double dt = 0.1;

// Predict state after latency
// x, y and psi are all zero after transformation above
double pred_px = 0.0 + v * dt; // Since psi is zero, cos(0) = 1, can leave out
const double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
double pred_psi = 0.0 + v * -delta / Lf * dt;
double pred_v = v + a * dt;
double pred_cte = cte + v * sin(epsi) * dt;
double pred_epsi = epsi + v * -delta / Lf * dt;
```

### Solve for new actuations

```cpp
// Feed in the predicted state values
Eigen::VectorXd state(6);
state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

// Solve for new actuations (and to show predicted x and y in the future)
auto vars = mpc.Solve(state, coeffs);
```

### Calculate steering and throttle

Steering must be divided by deg2rad(25) to normalize within [-1, 1].

Multiplying by Lf takes into account vehicle's turning ability

```cpp
double steer_value = vars[0] / (deg2rad(25) * Lf);
double throttle_value = vars[1];
```


## mpc.cpp

-> [CarND-18-Motion-Planning-MPC-Model-Predictive-Control](https://github.com/ChenBohan/CarND-18-Motion-Planning-MPC-Model-Predictive-Control)
