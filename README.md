# FTC PID Class
PID implimentation for FTC.
---
## Usage
`PID::PID(double P, double I, double D, double sp)`
Constructs a new PID controller.

Parameters:
  P   The proportional constant.
  I   The integral constant.
  D   The derivative constant.
  sp  The initial set point.
---

`public double PIDLoop(double currentPos)`
Returns the process variable.  The process varaible is directly fed to the feedback system (Ex. Motor).  Should be called every loop.

Parameters:
  currentPos  The process variable that is fed to the system.
---

`public void setSetPoint(double sp)`
Updates the set point.

Parameters:
  sp  New value for the set point.
---

`public void updateConst(double P, double I, double D)`
Updates the PID constants.

Parameters:
  P   The proportional constant.
  I   The proportional constant.
  D   The proportional constant.
