The codes in `ctrRLWE` implements an encrypted dynamic controller through the method of [this paper](https://ieeexplore.ieee.org/abstract/document/10730788).
The documentation for these codes can be found here (to be updated).

---

### Overview
- `conversion.m`: This MATLAB code designs a controller that stabilizes a discretized four-tank system. Then, it converts the controller to an input-output form.
    It also plots the performance of the converted and then quantized controller.
    The parameters obtained from this code are used in `main.go`.
- `main.go`: This Go code encrypts the pre-designed controller from `conversion.m` and runs the encrypted control system.
    It generates five csv files:
  - `state.csv`: This contains the state data of the plant controlled by the encrypted controller.
  - `uEnc.csv`: This saves the control input from the encrypted controller. These are the plaintexts which are injected as the plant input.
  - `yEnc.csv`: This saves the plant output of the encrypted control system.
  - `uDiff.csv`: This signifies the performance error, which is the norm of the difference between the control inputs from the encrypted controller and the pre-designed controller.
  - `period.csv`: The elapsed time is measured for each control period.

---

### Key variables
(to be updated)
