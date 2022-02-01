### Controller Module

For controller in our project, PID controller and Stanley method was employed. 

PID controller was used for longitudinal control, where the error between current velocity and desired velocity is the control input, while the acceleration of the vehicle is the control output. 
The act of throttling and braking will be performed accordingly to the output acceleration. 

Stanley controller was used for lateral control, where final expected steering depends on heading error and and cross-track error of the driving vehicle. 

