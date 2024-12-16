# Contributions

## Daniel
### TinyMPC

[TinyMPC](https://github.com/Astronauty/acsi-team5-2024/blob/main/_thirdparty/crazyflie-firmware/src/modules/src/controller/controller_mpc.c): 
A barebones TinyMPC controller was created by instantiating another controller directly in the crazyflie-firmware (as opposed to implementing out-of-tree, since we had issues with the compiler toolchain recognizing the out-of-tree builds). Under the existing controller framework, this makes it drop-in compatible with reference commands sent via the Python Commander API.

[TinyMPC CodeGen](https://github.com/Astronauty/acsi-team5-2024/blob/main/tinympc_codegen_quadrotor.py)
CodeGen for TinyMPC.

[OSQP](https://github.com/Astronauty/acsi-team5-2024/blob/main/osqp_hover.py)
Solution to LQR tracking with primal constraints using OSQP.

[Refueling Orchestrator](https://github.com/Astronauty/acsi-team5-2024/blob/main/refueling_orchestrator.py)
High-level interface using the commander class for the refueling problem. Contains a user-triggered state machine for changing the reference passed to the low level controller relative to the Tumbller.

[Tumbller Velocity Integrator](https://github.com/Astronauty/acsi-team5-2024/blob/main/tumbller_state_predictor.py)
Simple integrator to predict future position of Tumbller based on linear interpolation of current velocity.

## Tianqi
### Webot
The system's simulation environment was built in Webot for both Tumbller and Crazyfile. The controller for TUmbller and Crazyfile, which are written in Python, can be found in the ACSI_simulation -- Controller folder. Webot world can be seen and modified in the ACSI_simulation -- Worlds folder.
### Hardware
## Jonathan S.
## Jonathan C.

# Crazyflie2.1 Setup 

This guide will walk you through setting up a Windows 11 environment or WSL to run Crazyflie2.1 examples and cfclient.

## Prerequisites
- Ensure you have [Anaconda](https://www.anaconda.com/products/individual) installed, or use Python's built-in `venv` if preferred.
- Python 3.10 is required for the Crazyflie client.
## Instructions

1. **Set up a virtual environment with Python 3.10. Choose one method or the other** 

   ### Anaconda Environment
   Open Anaconda Prompt (or any terminal with conda available) and create a new environment:
   ```bash
   conda create -n crazyflie python=3.10
   conda activate crazyflie
   ```

   ### Python Built-in `venv`
   Alternatively, for `venv`:
      ```bash
      python -m venv cfvenv
      ```
    - Activating in Windows:
      ```bash
      .\cfvenv\Scripts\activate
      ```
    - Activating in Linux:
      ```bash
      call cfvenv\Scripts\activate
        ```

2. **Install the Crazyflie client**

   Install `cfclient` through pip:
   ```bash
   pip install cfclient
   ```

3. **Install the Crazyflie2 radio dongle USB driver**

   Follow the [Bitcraze documentation](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/) for setting up the USB driver for the Crazyflie2 radio dongle on Windows.

4. **Running the cfclient**

   Launch the Crazyflie client:
   ```bash
   cfclient
   ```

## Testing the Setup

To ensure everything is set up correctly, connect the Crazyflie2.1 and attempt to connect via the `cfclient` GUI. If you encounter issues with connection, verify that the USB radio dongle driver is correctly installed and recognized by the client.

## Additional Resources
- [Crazyflie2.1 Documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/getting-started/)
- [cfclient Source Code](https://github.com/bitcraze/crazyflie-clients-python)
