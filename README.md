# Contributions

## Daniel
### TinyMPC
    [a relative link] (acsi-team5-2024/_thirdparty/crazyflie-firmware/src/modules/src/controller/controller_mpc.c)
### OSQP
### Refueling Orchestrator

## Tianqi
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
