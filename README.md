# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Purpose

The purpose of this project was to get a feel on the physics of how to tune a PID controller while also gaining experience with implementing a PID controller in C++ code.

## Strategy

Initially, I tried to implement twiddle but my twiddle methodology needs fixing since I am only able to tune only two values such as P and D.  And even then the values are not tuned properly.  For example, using this strategy it sets both P and D to values such as 15+ which is way too large for this system.  However, the car is able to stay on track the whole time except it just oscillates at a very high frequency while traveling very slowly.  But this is definitely not an ideal ride for humans!

This is when I decided to just do manual tuning.   First I also did a bit of research.  I referred to the following website for some strategies:

https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops

Here was some good advice I found from the above website.  I will paste it below:

To tune a PID use the following steps:

1) Set all gains to zero.
2) Increase the P gain until the response to a disturbance is steady oscillation.
3) Increase the D gain until the the oscillations go away (i.e. it's critically damped).
4) Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
5) Set P and D to the last stable values.
6) Increase the I gain until it brings you to the setpoint with the number of oscillations desired (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot)

Taking the above strategy into consideration, first I initialized everything to zero and increased the P until I had some small oscillations.  This took some trial and error.  I started in the 0.01 range and increased slowly to maybe about 0.04.  The car would drive fine, but then I noticed that oscillations got larger and larger.  This meant we need to increase the D term!

Therefore, I increased the D term to dampen the oscillations and this seemed to work well but it was not complete.  This brought me to a point where where the car would just kind touch the edges but still stay on the road because it was using the walls. So, I just tried to increase the I a very tiny value such as 0.001.  This improved a lot but was not enough!   So, then I decided to increase the P value to give it more steering strength on the turns to get keep it from hitting the rails.  But then I thought maybe there might be still be some cumulative bias that needed to be corrected so at this point I doubled the 'I term' to 0.002.  Now it looks like the vehicle passes and I have now decided to just do a submission to see how does it look.

## Reflections

The effects of P, I, D are expected.   When I only have the P parameter I see the car oscillate and the oscillations start small and increase especially near the point where the vehicle turns.  The D term helps counter-steer to prevent these huge oscillations as expected.  And the I term eliminates the long-term bias.  Basically, the P term is the present state.  The I term covers the past states.  The D term covers the future states.

From wikipedia there was a great animated gif that basically described my process.  It also provides a very cool visual as what you are doing while tuning the parameters.  My strategy was to tune, P and D before I.  But in the gif below it first shows the tuning of P, then I and finally D:

https://en.wikipedia.org/wiki/File:PID_Compensation_Animated.gif

As a reminder in case the reader stepped away and had a twinkle while doing twiddle.  Refer to the strategy section above to see how I tuned the parameters.

## Future work

I want to get twiddle fully implemented so that manual tuning does not have to take place.  I have my skeleton code in place and commented out which I will keep there as a possible exploration strategy.  Although I do believe it was a great exercise to do the manual tuning because this helps provide the practitioner with better intuition.
