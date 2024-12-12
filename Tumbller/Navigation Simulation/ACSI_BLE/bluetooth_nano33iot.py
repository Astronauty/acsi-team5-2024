'''
Make sure to install these depencies:
!pip install bleak 
!pip install cflib
'''
import asyncio
import struct
from bleak import BleakClient
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# Define the Bluetooth address and UUIDs
address = "e0:5a:1b:79:89:42"
service_uuid = "180C"  
characteristic_uuid = "2A56"  

# Initialize the low-level drivers
cflib.crtp.init_drivers(enable_debug_driver=False)

# URI to the Crazyflie
uri = 'radio://0/80/2M'

# Data to send (example uint8_t array to read)
'''
data = bytearray([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
                  32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47])
'''
data = bytearray(48)

async def send_data(address, service_uuid, characteristic_uuid, data):
    async with BleakClient(address) as client:
        # Ensure the client is connected
        if not client.is_connected:
            print("Failed to connect to the device.")
            return

        print("Connected to the device.")

        while True:
            # Write data to the characteristic
            await client.write_gatt_char(characteristic_uuid, data)
            print("Data sent:", data)

            # Optionally, read back the data to verify
            response = await client.read_gatt_char(characteristic_uuid)
            print("Received response:", response)

            # Send data every 10ms
            await asyncio.sleep(0.01)

def log_pos_callback(timestamp, data, logconf):
    global x, y, theta, x_dot, y_dot, theta_dot
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    theta = data['stateEstimate.yaw']
    x_dot = data['stateEstimate.vx']
    y_dot = data['stateEstimate.vy']
    theta_dot = data['stateEstimate.r']

    # Pack the data into the bytearray
    data[0:8] = bytearray(struct.pack('f', x))
    data[8:16] = bytearray(struct.pack('f', y))
    data[16:24] = bytearray(struct.pack('f', theta))
    data[24:32] = bytearray(struct.pack('f', x_dot))
    data[32:40] = bytearray(struct.pack('f', y_dot))
    data[40:48] = bytearray(struct.pack('f', theta_dot))

async def main():
    global data

    # Connect to the Crazyflie
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        log_conf = LogConfig(name='Position', period_in_ms=10)
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.yaw', 'float')
        log_conf.add_variable('stateEstimate.vx', 'float')
        log_conf.add_variable('stateEstimate.vy', 'float')
        log_conf.add_variable('stateEstimate.r', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(log_pos_callback)
        log_conf.start()

        await send_data(address, service_uuid, characteristic_uuid, data)

if __name__ == "__main__":
    asyncio.run(main())



















### Plotting diagnostics to determine communication bottleneck of system
# import numpy as np
# import matplotlib.pyplot as plt

# latencies = np.array([305,294,286,274,314,296,
#                       288,310,361,301,303,356,
#                       282,311,285,294,302,398,
#                       288,291,298,389,328,351,
#                       293,374,349,279])

# average = np.mean(latencies)

# control_inputs = np.array([-0.30,-0.41,-0.33,-0.65,-0.63,-0.02,0.34,0.27,
#                           0.25,0.26,0.67,0.31,-0.00,-0.32,-0.48,-0.18,
#                           -0.24,-0.45,-0.71,-0.26,0.13,0.36,0.30,-0.05,
#                           -0.16,-0.06,0.29,0.54,0.20,-0.16,-0.41,-0.34,
#                           -0.31,-0.53,-0.64,-0.16,0.14,0.32,0.14,-0.02,
#                           0.08,0.48,0.36,0.11,-0.28,-0.41,-0.27,-0.23,
#                           -0.44,-0.86,-0.89,-0.40,1.49,2.64,0.89,0.11,
#                           0.35,0.88,0.78,1.05,1.61,-3.39,-4.45,-0.73,-0.98,
#                           0.58,-0.24,-1.23,-3.55,-4.49,-1.46,4.77,3.69,-0.08,
#                           -4.77,-2.44,2.41,2.19,-1.17,-3.11,-1.34,2.52,
#                           -1.66,-1.73,-0.44,1.15,0.48,-1.56,-1.75,-0.16,
#                           1.14,-0.70,-1.55,-0.29,0.37,-0.07,-0.39,-0.23,
#                           -0.23,-0.41,-0.48,-0.37,-0.21,-0.21,-0.14,-0.17,
#                           -0.22,0.36,-0.08,-1.04,-1.72])

# average = np.mean(control_inputs)

# # print the average latency in the figure
# plt.plot(control_inputs, 'o')
# plt.axhline(y=average, color='r', linestyle='-')
# plt.text(0, average, 'Average: {:.2f}'.format(average), color='r')
# plt.xlabel('Sample')
# plt.ylabel('Latency (µs)')
# plt.title('Bluetooth Latency')
# plt.show()


