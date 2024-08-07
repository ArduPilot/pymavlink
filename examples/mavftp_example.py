#!/usr/bin/env python3

'''
MAVLink File Transfer Protocol support example

SPDX-FileCopyrightText: 2024 Amilcar Lucas

SPDX-License-Identifier: GPL-3.0-or-later
'''

from argparse import ArgumentParser

from logging import basicConfig as logging_basicConfig
from logging import getLevelName as logging_getLevelName

from logging import info as logging_info
from logging import warning as logging_warning
from logging import error as logging_error

import sys
import os
import requests
import time

from typing import Tuple

from pymavlink import mavutil
from pymavlink import mavftp


if __name__ == "__main__":

    def argument_parser():
        """
        Parses command-line arguments for the script.

        This function sets up an argument parser to handle the command-line arguments for the script.

        Returns:
        argparse.Namespace: An object containing the parsed arguments.
        """
        parser = ArgumentParser(description='This main is for testing and development only. '
                                'Usually, the mavftp is called from another script')
        parser.add_argument("--baudrate", type=int,
                        help="master port baud rate", default=115200)
        parser.add_argument("--device", required=True, help="serial device")
        parser.add_argument("--source-system", dest='SOURCE_SYSTEM', type=int,
                        default=250, help='MAVLink source system for this GCS')
        parser.add_argument("--loglevel", default="INFO", help="log level")
        parser.add_argument("--filename", default="@PARAM/param.pck?withdefaults=1", help="file to fetch")
        parser.add_argument("--decode-parameters", action='store_true', help="decode as a parameter file")
        parser.add_argument("--output-parameters-to-files", action='store_true', help="output parameters to files")
        parser.add_argument('-s', '--sort',
                            choices=['none', 'missionplanner', 'mavproxy'],
                            default='missionplanner',
                            help='Sort the parameters in the file. Defaults to %(default)s.',
                            )

        return parser.parse_args()


    def wait_heartbeat(m):
        '''wait for a heartbeat so we know the target system IDs'''
        logging_info("Waiting for flight controller heartbeat")
        m.wait_heartbeat()
        logging_info("Got heartbeat from system %u, component %u", m.target_system, m.target_system)


    def delete_local_file_if_exists(filename):
        if os.path.exists(filename):
            os.remove(filename)


    def missionplanner_sort(item: str) -> Tuple[str, ...]:
        """
        Sorts a parameter name according to the rules defined in the Mission Planner software.
        """
        return tuple(item.split("_"))


    def extract_params(pdata, sort_type):
        pdict = {}
        if pdata:
            for (name, value, _ptype) in pdata:
                pdict[name.decode('utf-8')] = value

            if sort_type == "missionplanner":
                pdict = dict(sorted(pdict.items(), key=lambda x: missionplanner_sort(x[0])))  # sort alphabetically
            elif sort_type == "mavproxy":
                pdict = dict(sorted(pdict.items()))  # sort in ASCIIbetical order
            elif sort_type == "none":
                pass
        return pdict


    def save_params(pdict, filename, sort_type):
        if not pdict:
            return
        with open(filename, 'w', encoding='utf-8') as f:
            for name, value in pdict.items():
                if sort_type == "missionplanner":
                    f.write(f"{name},{format(value, '.6f').rstrip('0').rstrip('.')}\n")
                elif sort_type == "mavproxy":
                    f.write(f"{name:<16} {value:<8.6f}")
                elif sort_type == "none":
                    f.write(f"{name:<16} {value:<8.6f}")
        logging_info("Outputted %u parameters to %s", len(pdict), filename)


    def get_params(args, mav_ftp):
        def get_param_callback(fh):
            '''called on ftp completion'''
            if fh is None:
                return
            data = fh.read()
            logging_info("param get done! got %u bytes", len(data))
            if args.decode_parameters or args.output_parameters_to_files:
                pdata = mavftp.MAVFTP.ftp_param_decode(data)
                if pdata is None:
                    logging_error("param decode failed")
                    sys.exit(1)

                pdict = extract_params(pdata.params, args.sort)
                defdict = extract_params(pdata.defaults, args.sort)

            if args.decode_parameters:
                for name, value in pdict.items():
                    if name in defdict:
                        print(f"{name:<16} {value} (default {defdict[name]})")
                    else:
                        print(f"{name:<16} {value}")

            if args.output_parameters_to_files:
                save_params(pdict, 'params.param', args.sort)
                save_params(defdict, 'defaults.param', args.sort)

        mav_ftp.cmd_get([args.filename], callback=get_param_callback)
        mav_ftp.execute_ftp_operation()


    def get_list_dir(mav_ftp, directory):
        mav_ftp.cmd_list([directory])
        mav_ftp.execute_ftp_operation()


    def get_file(mav_ftp, remote_filename, local_filename, timeout=5):
        logging_info("Will download %s to %s", remote_filename, local_filename)
        mav_ftp.cmd_get([remote_filename, local_filename])
        mav_ftp.execute_ftp_operation(timeout=timeout)


    def get_last_log(mav_ftp):
        try:
            with open('LASTLOG.TXT', 'r', encoding='UTF-8') as file:
                remote_filenumber = int(file.readline().strip())
        except FileNotFoundError:
            logging_error("File LASTLOG.TXT not found.")
            return
        remote_filenumber = remote_filenumber - 1 # we do not want the very last log
        remote_filename = f'/APM/LOGS/{remote_filenumber:08}.BIN'
        get_file(mav_ftp, remote_filename, 'LASTLOG.BIN', 500)
        time.sleep(1) # wait for the file to be written to disk and for the FC to catch it's breath


    def download_script(url, local_filename):
        # Download the script from the internet to the PC
        response = requests.get(url, timeout=5)

        if response.status_code == 200:
            with open(local_filename, "wb") as file:
                file.write(response.content)
        else:
            logging_error("Failed to download the file")


    def create_directory(mav_ftp, remote_directory):
        mav_ftp.cmd_mkdir([remote_directory])
        mav_ftp.execute_ftp_operation()


    def upload_script(mav_ftp, remote_directory, local_filename):
        # Upload it from the PC to the flight controller
        mav_ftp.cmd_put([local_filename, remote_directory])
        mav_ftp.execute_ftp_operation()
        time.sleep(0.3) # wait for the FC to catch it's breath


    def main():
        '''for testing/example purposes only'''
        args = argument_parser()

        logging_basicConfig(level=logging_getLevelName(args.loglevel), format='%(asctime)s - %(levelname)s - %(message)s')

        logging_warning("This main is for testing and development only. "
                        "Usually the mavftp is called from another script")

        # create a mavlink serial instance
        master = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM)

        # wait for the heartbeat msg to find the system ID
        wait_heartbeat(master)

        mav_ftp = mavftp.MAVFTP(master,
                                target_system=master.target_system,
                                target_component=master.target_component)

        delete_local_file_if_exists("params.param")
        delete_local_file_if_exists("defaults.param")
        get_params(args, mav_ftp)

        get_list_dir(mav_ftp, '/APM/LOGS/')
        
        delete_local_file_if_exists("LASTLOG.TXT")
        delete_local_file_if_exists("LASTLOG.BIN")
        get_file(mav_ftp, '/APM/LOGS/LASTLOG.TXT', 'LASTLOG.TXT')

        get_last_log(mav_ftp)

        remote_directory = '/APM/Scripts/'
        #create_directory(mav_ftp, remote_directory)

        url = "https://discuss.ardupilot.org/uploads/short-url/4pyrl7PcfqiMEaRItUhljuAqLSs.lua"
        local_filename = "copter-magfit-helper.lua"

        if not os.path.exists(local_filename):
            download_script(url, local_filename)

        upload_script(mav_ftp, remote_directory, local_filename)

        url = "https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5/libraries/AP_Scripting/applets/VTOL-quicktune.lua"
        local_filename = "VTOL-quicktune.lua"

        if not os.path.exists(local_filename):
            download_script(url, local_filename)

        upload_script(mav_ftp, remote_directory, local_filename)
        
        master.close()


    main()
