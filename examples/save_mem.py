# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Example of how to read the Lighthouse base station geometry and
calibration memory from a Crazyflie
"""
import logging
from threading import Event

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseMemHelper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class ReadMem:
    def __init__(self, uri, output_file):
        self._event = Event()
        self.output_file = output_file

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            helper = LighthouseMemHelper(scf.cf)

            helper.read_all_geos(self._geo_read_ready)
            self._event.wait()

            self._event.clear()

            helper.read_all_calibs(self._calib_read_ready)
            self._event.wait()

    def _geo_read_ready(self, geo_data):
        with open(self.output_file, 'a') as file:
            for id, data in geo_data.items():
                file.write(f'---- Geometry for base station {id + 1}\n')
                file.write(f'origin: {data.origin}\n')
                file.write(f'rotation matrix: {data.rotation_matrix}\n')
                file.write(f'valid: {data.valid}\n\n')
        self._event.set()

    def _calib_read_ready(self, calib_data):
        with open(self.output_file, 'a') as file:
            for id, data in calib_data.items():
                file.write(f'---- Calibration data for base station {id + 1}\n')
                for sweep in data.sweeps:
                    file.write(f'phase: {sweep.phase}, tilt: {sweep.tilt}, curve: {sweep.curve}, '
                               f'gibmag: {sweep.gibmag}, gibphase: {sweep.gibphase}, '
                               f'ogeemag: {sweep.ogeemag}, ogeephase: {sweep.ogeephase}\n')
                file.write(f'uid: {data.uid}\n\n')
                file.write(f'valid: {data.valid}\n\n')
        self._event.set()


if __name__ == '__main__':
    # URI to the Crazyflie to connect to
    #uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    uri = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E702')

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Specify the output file
    output_file = 'lighthouse_memory.txt'

    # Read memory and save to file
    print("Reading ligthouse memory")
    ReadMem(uri, output_file)
    print("Saving done. Run the 'write_mem.py' example to write the data to a new crazyflie")

