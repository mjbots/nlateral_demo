# moteus n-lateral control demonstration #

This is a standalone project that demonstrates high-rate n-lateral
control of 2 or more moteus servos.  All configured servos are linked
into a "virtual geartrain" with force and translation individually
configurable.

From the help:

```
Usage: nlateral_demo [options]

  -h, --help           display this usage message
  --main-cpu CPU       run main thread on a fixed CPU [default: 1]
  --can-cpu CPU        run CAN thread on a fixed CPU [default: 2]
  --period-s S         period to run control
  --kp XX.X            select kp value
  --kd XX.X            select kd value
  --max-torque XX.X    maximum torque to apply to a servo
  -s, --servo CFG      add one servo to be controlled
   CFG=ID[,option]...
    bN - pi3hat bus number N (default 1)
    pXX.X - scale position by this positive float
    fXX.X - scale force by this positive float

Example w/ two moteus devkit motors on ID 1 and 2:
  sudo ./nlateral_demo -s 1 -s 2 --period-s 0.001 --kp 1.0 --kd 0.01
```


WARNING: It is easy to select parameters that result in an unstable
system.  It is advisable to both use a current limited power supply,
and securely fasten any motors to rigid surfaces so that in the event
of instability they do not jump around.
