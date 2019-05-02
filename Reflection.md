# Reflections

To implement the PID controller is easy. The only thing that need to be taken care of it the first value of d_error, which should be zero. So I add a 'initialized' bool variable to indicate whether the pid controller is just initialized or not.

To determine the coefficient of pid controller takes some efforts. I start with setting Kp to 1. Kp serves as a proportional coefficient. Simply speaking, it corrects the error based on how large the error is.

And result is the vehicle keep turning right and just run out of track. By seeing the output of CTE, I can see that CTE keep increasing. So the Kp should be a negative value. I also noticed that the,
- initial CTE is about 0.7
- the width of the track is about 4 
- the range of `steering_value` is `[-1, 1]`

I set the value of Kp to -0.1, so that that steering is not much too sensitive to the CTE. Now the K is `[-0.1, 0, 0]`.

The vehicle runs smoothly in the center at the beginning. However, as the speed goes up to 20+, the vehicle becomes very unstable and runs out of track eventually. 

But before doing that, I'd like control the speed first. Because I want the vehicle to complete the track first, so that I can calculate the error of the complete track. Here I just use a simple control as following, instead of another PID controller.

```C++
if(speed < 10.0) {
    throttle_value = 0.3;
}
else {
    throttle_value = 0;
}
```

So now the vehicle will run at 10mph. Re-run the simulator with `[-0.1, 0, 0]`. Regardless of some breathtaking moments at the corners, it can finish a whole lap. That's a great start. 

Next, I will try to implement an evaluation funciton, using mean squared error(MSE). This can be useful for twiddle. It is needed to know how many data points in a lap and the sum or squared errors. It is noteworthy that the number of data points in a lap is different at different speed. For example, there're ~4900 datapoints at 10mph, and ~3100 at 15mph.

For `[-0.1, 0, 0]` at 10mph, I get 1.09614 for MSE. Now it's time to consider Ki and Kd. 

Ki is the integral coefficient. It can provide compensation based on history error. For example, if the vehicle is in the curve and the steering angle is not big enough to bring the vehicle back to center, integral error can help to provide more steering angle.

Kd is the derivative coefficient. It takes rate of change of error into consideration. It helps the controller not to overshoot. 

So here I play with different combinations of Kp, Ki and Kp. The results are:

Coefficients of PID    | MSE
---------------------- | -------------
[-0.1, 0, 0]           | 1.09614
[-0.1 ,0, -0.1]        | 1.05055
[-0.1, -0.0001, -0.2]  | 0.786436
[-0.15, -0.0001, -0.2] | 0.375419
[-0.16, -0.0001, -0.2] | 0.340783

I can keep tuning the parameters until it performs good enough. But it takes too long time and and it's tedious. Now I will try to use twiddle with speed control.

To enable Twiddle, I use a variable `dK` to represent the tuning values of K. The basic rule is, when dK reaches theshold, increase speed by 2, reset best_error, dK and start twiddle all over again.

The following is a snapshoot of the debug print of the process. 

```
------------------------------
Twiddle session: #0     TState:1        Para:0
Current K:[-0.181, -0.000121, -0.221]
Error:0.323396  Best error:0.323396
Updated K:[-0.171, -0.000121, -0.221]
------------------------------
Twiddle session: #1     TState:2        Para:0
Current K:[-0.171, -0.000121, -0.221]
Error:0.344201  Best error:0.323396
Updated K:[-0.191, -0.000121, -0.221]
------------------------------
Twiddle session: #2     TState:3        Para:0
Current K:[-0.191, -0.000121, -0.221]
Error:0.285942  Best error:0.323396
Updated K:[-0.191, -0.000111, -0.221]
------------------------------
Twiddle session: #3     TState:2        Para:1
Current K:[-0.191, -0.000111, -0.221]
Error:0.285802  Best error:0.285942
Updated K:[-0.191, -0.000131, -0.221]
------------------------------
Twiddle session: #4     TState:3        Para:1
Current K:[-0.191, -0.000131, -0.221]
Error:0.277162  Best error:0.285942
Updated K:[-0.191, -0.000131, -0.211]
------------------------------
Twiddle session: #5     TState:2        Para:2
Current K:[-0.191, -0.000131, -0.211]
Error:0.281714  Best error:0.277162
Updated K:[-0.191, -0.000131, -0.231]
------------------------------
Twiddle session: #6     TState:3        Para:2
Current K:[-0.191, -0.000131, -0.231]
Error:0.284975  Best error:0.277162
Updated K:[-0.18, -0.000131, -0.221]
------------------------------
Twiddle session: #7     TState:2        Para:0
Current K:[-0.18, -0.000131, -0.221]
Error:0.307973  Best error:0.277162
Updated K:[-0.202, -0.000131, -0.221]
------------------------------
Twiddle session: #8     TState:3        Para:0
Current K:[-0.202, -0.000131, -0.221]
Error:0.270575  Best error:0.277162
Updated K:[-0.202, -0.00012, -0.221]
------------------------------
Twiddle session: #9     TState:2        Para:1
Current K:[-0.202, -0.00012, -0.221]
Error:0.266286  Best error:0.270575
Updated K:[-0.202, -0.000142, -0.221]
...

Twiddle session: #59    TState:2        Para:2
Current K:[-0.202, -0.000142, -0.217126]
Error:0.27025   Best error:0.250613
Updated K:[-0.202, -0.000142, -0.224874]
```

Each loop takes about 3~4min depending on the speed. Unfortunately there's no faster simulation tool. I spend hours runing the simulator. And the result I get so far is [-0.202, -0.000142, -0.221] at 15mph. The best error is 0.250613. If the twiddle process keeps running, I believe the car would run better at a higher speed.











