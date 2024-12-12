# -*- coding: utf-8 -*-
import logging
from threading import Event
import ast  # To parse string representations of Python objects
import argparse

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsCalibration, LighthouseBsGeometry
from cflib.crazyflie.mem import LighthouseMemHelper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def parse_file(file_path):
    """
    Parse the lighthouse_memory.txt file and return dictionaries for geometry and calibration data.
    """
    geo_dict = {}
    calib_dict = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()

    base_station = None
    sweep_index = 0  # Tracks which sweep we are parsing (0 or 1)
    for line in lines:
        if 'Geometry for base station' in line:
            base_station = int(line.strip().split()[-1]) - 1
            geo_dict[base_station] = LighthouseBsGeometry()
        elif 'origin:' in line and base_station is not None:
            geo_dict[base_station].origin = ast.literal_eval(line.split(':', 1)[1].strip())
        elif 'rotation matrix:' in line and base_station is not None:
            geo_dict[base_station].rotation_matrix = ast.literal_eval(line.split(':', 1)[1].strip())
        elif 'valid:' in line and base_station is not None:
            geo_dict[base_station].valid = line.split(':', 1)[1].strip() == 'True'
        elif 'Calibration data for base station' in line:
            base_station = int(line.strip().split()[-1]) - 1
            calib_dict[base_station] = LighthouseBsCalibration()
            sweep_index = 0  # Reset sweep index for the new base station
        elif 'phase:' in line and base_station is not None:
            # Extract key-value pairs for this sweep
            parts = line.strip().split(', ')
            for part in parts:
                key, value = part.split(': ')
                if key == 'phase':
                    calib_dict[base_station].sweeps[sweep_index].phase = float(value)
                elif key == 'tilt':
                    calib_dict[base_station].sweeps[sweep_index].tilt = float(value)
                elif key == 'curve':
                    calib_dict[base_station].sweeps[sweep_index].curve = float(value)
                elif key == 'gibmag':
                    calib_dict[base_station].sweeps[sweep_index].gibmag = float(value)
                elif key == 'gibphase':
                    calib_dict[base_station].sweeps[sweep_index].gibphase = float(value)
                elif key == 'ogeemag':
                    calib_dict[base_station].sweeps[sweep_index].ogeemag = float(value)
                elif key == 'ogeephase':
                    calib_dict[base_station].sweeps[sweep_index].ogeephase = float(value)
            sweep_index += 1  # Increment to the next sweep
        elif 'uid:' in line and base_station is not None:
            calib_dict[base_station].uid = int(line.split(':', 1)[1].strip())
        elif 'valid:' in line and base_station is not None:
            calib_dict[base_station].valid = line.split(':', 1)[1].strip() == 'True'

    return geo_dict, calib_dict


class WriteMem:
    def __init__(self, uri, geo_dict, calib_dict):
        self._event = Event()

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            helper = LighthouseMemHelper(scf.cf)

            helper.write_geos(geo_dict, self._data_written)
            self._event.wait()

            self._event.clear()

            helper.write_calibs(calib_dict, self._data_written)
            self._event.wait()

    def _data_written(self, success):
        if success:
            print('Data written')
        else:
            print('Write failed')

        self._event.set()


if __name__ == '__main__':

    args = argparse.ArgumentParser()
    args.add_argument("--id", type=int, default=1, help="Crazyflie ID")
    args.add_argument("--file ", type=str, default="lighthouse_memory.txt", help="File path to read memory data from")
    args = args.parse_args()

    # URI to the Crazyflie to connect to
    radio = 'radio://0/20/2M/E7E7E7E7{:02d}'.format(args.id)
    uri = uri_helper.uri_from_env(default=radio)

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Parse the memory data from the file
    print("Parsing memory data from lighthouse_memory.txt")
    geo_dict, calib_dict = parse_file('lighthouse_memory.txt')

    # Output the parsed dictionaries
    print("Parsed geo_dict:")
    for key, value in geo_dict.items():
        print(f"Base Station {key}: {value.origin}, {value.rotation_matrix}, {value.valid}")

    print("\nParsed calib_dict:")
    for key, value in calib_dict.items():
        print(f"Base Station {key}: {value.uid}, {value.sweeps[0].phase}, {value.valid}")

    # Write memory data to the new Crazyflie
    WriteMem(uri, geo_dict, calib_dict)
    print("Memory data written to the Crazyflie")
