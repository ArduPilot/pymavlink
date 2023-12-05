# Contents

| Tool                      | Description                                                                                                  |
| ------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `AccelSearch`             | Search a set of log files for bad accel values                                                               |
| `extract_batch_imu`       | Extract ISBH and ISBD messages from AP_Logging files into a bin GYR and ACC log                              |
| `extract_parms`           | Extract non-default parameters for publishing                                                                |
| `find_aliasing`           | Find signs of aliasing on IMU3                                                                               |
| `isb_extract`             | Extract ISBH and ISBD messages from AP_Logging files into a csv file                                         |
| `magfit_compassmot`       | Estimate COMPASS_MOT\_\* parameters for throttle based compensation                                          |
| `magfit_delta`            | Fit best estimate of magnetometer offsets using the algorithm from Bill Premerlani                           |
| `magfit_elliptical`       | Fit best estimate of magnetometer offsets                                                                    |
| `magfit_gps`              | Fit best estimate of magnetometer offsets                                                                    |
| `magfit_motors`           | Fit best estimate of magnetometer offsets, trying to take into account motor interference                    |
| `magfit_rotation_gps`     | Fit best estimate of magnetometer rotation to GPS data                                                       |
| `magfit_rotation_gyro`    | Fit best estimate of magnetometer rotation to gyro data                                                      |
| `magfit_WMM`              | Fit best estimate of magnetometer offsets, diagonals, off-diagonals, cmot and scaling using WMM target       |
| `magfit`                  | Fit best estimate of magnetometer offsets                                                                    |
| `mavextract`              | Extract one mode type from a log                                                                             |
| `mavfft_int`              | Interactively select accel and gyro data for FFT analysis                                                    |
| `mavfft_isb`              | Extract ISBH and ISBD messages from AP_Logging files and produce FFT plots                                   |
| `mavfft_pid`              | Fit estimate of PID oscillations                                                                             |
| `mavfft`                  | Fit best estimate of magnetometer offsets                                                                    |
| `mavflightmodes`          | Show changes in flight modes                                                                                 |
| `mavflighttime`           | Work out total flight time for a mavlink log                                                                 |
| `mavftpdecode`            | Decode FTP file transfers from tlog                                                                          |
| `mavgen`                  | Parse a MAVLink protocol XML file and generate a python implementation                                       |
| `mavgps_CEP`              | Calculate GPS CEP from DF or mavlink log for all present GPS modules                                         |
| `mavgpslag`               | Calculate GPS lag from DF log                                                                                |
| `mavgpslock`              | Show GPS lock events in a MAVLink log                                                                        |
| `mavgraph`                | Graph a MAVLink log file                                                                                     |
| `mavkml`                  | Simple kml export for logfiles                                                                               |
| `mavlink_bitmask_decoder` | Decodes the bitmask for a specific value in a message                                                        |
| `mavlink_messages_size`   | A dictionary containing the lengths of messages                                                              |
| `mavlogdump`              | Example program that dumps a Mavlink log file.                                                               |
| `mavloss`                 | Show MAVLink packet loss                                                                                     |
| `mavmerge`                | Merge two tlogs                                                                                              |
| `mavmission`              | Extract mavlink mission from log                                                                             |
| `mavmsgstats`             | Show stats on messages in a log                                                                              |
| `mavparmdiff`             | Compare two MAVLink parameter files                                                                          |
| `mavparms`                | Extract mavlink parameter values                                                                             |
| `mavplayback`             | Play back a mavlink log as a FlightGear FG NET stream, and as arealtime mavlink stream                       |
| `mavsearch`               | Search a set of log files for a condition                                                                    |
| `mavsigloss`              | Show times when signal is lost                                                                               |
| `mavsplit_sysid`          | Split log by system ID                                                                                       |
| `mavsummarize`            | Summarize MAVLink logs. Useful for identifying which log is of interest in a large set.                      |
| `mavtelemetry_datarates`  | GUI to calculate telemetry data rate between vehicle and GCS                                                 |
| `mavtogpx`                | Example program to extract GPS data from a mavlink log, and create a GPX file, for loading into google earth |
| `mavtomfile`              | Convert a MAVLink tlog file to a MATLab mfile                                                                |
| `MPU6KSearch`             | Search a set of log files for signs of inconsistent IMU data                                                 |
| `serial_control_to_shell` | Reads SERIAL_CONTROL packets and passes them to a shell, returning textual results                           |
| `sertotcp`                | Map a serial port to an outgoing TCP connection                                                              |
