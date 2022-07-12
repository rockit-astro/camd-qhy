## QHY camera daemon

`qhy_camd` interfaces with and wraps QHY600M detectors and exposes them via Pyro.

`cam` is a commandline utility for controlling the cameras.

See [Software Infrastructure](https://github.com/warwick-one-metre/docs/wiki/Software-Infrastructure) for an overview of the observatory software architecture and instructions for developing and deploying the code.

### Configuration

Configuration is read from json files that are installed by default to `/etc/camd`.
A configuration file is specified when launching the camera server, and the `cam` frontend will search for files matching the specified camera id when launched.

The configuration options are:
```python
{
  "daemon": "localhost_test", # Run the camera server as this daemon. Daemon types are registered in `warwick.observatory.common.daemons`.
  "pipeline_daemon": "localhost_test2", # The daemon that should be notified to hand over newly saved frames for processing.
  "pipeline_handover_timeout": 10, # The maximum amount of time to wait for the pipeline daemon to accept a newly saved frame. The exposure sequence is aborted if this is exceeded.
  "log_name": "qhy_camd@test", # The name to use when writing messages to the observatory log.
  "control_machines": ["LocalHost"], # Machine names that are allowed to control (rather than just query) state. Machine names are registered in `warwick.observatory.common.IP`.
  "camera_device_id": "QHY600M-2c52f645aa1c25b98", # Identifier reported by the SDK for the desired camera. If not known, set a dummy value and look at the list reported when camd attempts to connect.
  "cooler_setpoint": -5, # Default temperature for the CMOS sensor.
  "cooler_update_delay": 1, # Amount of time in seconds to wait between querying the camera temperature and cooling status.
  "cooler_pwm_step": 3, # PWM units to change per update delay when cooling/warming (3 = ~1%).
  "worker_processes": 3, # Number of processes to use for generating fits images and saving temporary images to disk.
  "mode": 0, # Camera read mode: 0 (photographic), 1 (high gain), 4 (14 bit readout).
  "gain": 26, # Gain setting for the CMOS sensor. See the QHY600 spec sheet for details on the implications on signal and read noise.
  "offset": 140, # Bias setting for the CMOS sensor.
  "use_gpsbox": true, # Use attached GPS Box to measure exposure timestamps.
  "header_card_capacity": 144, # Pad the fits header with blank space to fit at least this many cards without reallocation.
  "camera_id": "TEST", # Value to use for the CAMERA fits header keyword.
  "output_path": "/var/tmp/", # Path to save temporary output frames before they are handed to the pipeline daemon. This should match the pipeline incoming_data_path setting.
  "output_prefix": "test", # Filename prefix to use for temporary output frames.
  "expcount_path": "/var/tmp/test-counter.json" # Path to the json file that is used to track the continuous frame number.
}
```

### Initial Installation

The first step is to download and install the QHY Linux SDK from their website.

First run the `install.sh` script and then compile and install the PCI-e driver in `<sdk dir>/usr/local/riffa_linux_driver/`

The automated packaging scripts will push 4 RPM packages to the observatory package repository:

| Package           | Description |
| ----------------- | ------ |
| clasp-qhy-data  | Contains the json configuration files for the CLASP instrument. |
| qhy-camera-server | Contains the `qhy_camd` server and systemd service files for the camera server. |
| qhy-camera-client | Contains the `cam` commandline utility for controlling the camera server. |
| python3-warwick-qhy-camera | Contains the python module with shared code. |

The `qhy-camera-server` and `clasp-qhy-data` packages should be installed on the CLASP DAS machines, then the systemd service should be enabled:
```
sudo systemctl enable qhy_camd.service@<config>
sudo systemctl start qhy_camd@<config>
```

where `config` is the name of the json file for the appropriate camera.

Now open a port in the firewall so the TCS and dashboard machines can communicate with the camera server:
```
sudo firewall-cmd --zone=public --add-port=<port>/tcp --permanent
sudo firewall-cmd --reload
```

where `port` is the port defined in `warwick.observatory.common.daemons` for the daemon specified in the camera config.

The `qhy-camera-client` and `clasp-qhy-config` packages should be installed on the CLASP TCS machine for centralized control.

### Upgrading Installation

New RPM packages are automatically created and pushed to the package repository for each push to the `master` branch.
These can be upgraded locally using the standard system update procedure:
```
sudo yum clean expire-cache
sudo yum update
```

The daemon should then be restarted to use the newly installed code:
```
sudo systemctl stop qhy_camd@<config>
sudo systemctl start qhy_camd@<config>
```

### Testing Locally

The camera server and client can be run directly from a git clone:
```
./qhy_camd test.json
CAMD_CONFIG_ROOT=. ./cam test status
```
