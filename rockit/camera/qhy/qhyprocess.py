#
# This file is part of the Robotic Observatory Control Kit (rockit)
#
# rockit is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# rockit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with rockit.  If not, see <http://www.gnu.org/licenses/>.

"""Helper process for interfacing with the QHY SDK"""

# pylint: disable=too-many-arguments
# pylint: disable=too-many-instance-attributes
# pylint: disable=too-many-return-statements
# pylint: disable=too-many-branches
# pylint: disable=too-many-statements

from ctypes import c_double, c_int, c_uint8, c_uint32, c_void_p
from ctypes import byref, create_string_buffer, POINTER
import json
import pathlib
import platform
import sys
import threading
import time
import traceback
from astropy.time import Time
import astropy.units as u
import numpy as np
import Pyro4
from rockit.common import log
from .constants import CommandStatus, CameraStatus, CoolerMode


class QHYStatus:
    Success = 0
    Error = 0xFFFFFFFF


class QHYControl:
    GAIN = 6
    OFFSET = 7
    EXPOSURE = 8
    USBTRAFFIC = 12
    CURTEMP = 14
    CURPWM = 15
    MANUALPWM = 16
    COOLER = 18
    GPS = 36
    DDR = 48
    HUMIDITY = 62
    PRESSURE = 63
    UVLO_STATUS = 67


def open_device(driver, camera_device_id):
    """Returns None if device is not found"""
    # Enumerate cameras to find target
    camera_count = driver.ScanQHYCCD()
    print(f'found {camera_count} cameras')

    device_id_buf = create_string_buffer(32)
    for i in range(camera_count):
        status = driver.GetQHYCCDId(i, device_id_buf)
        if status != QHYStatus.Success:
            continue

        device_id = device_id_buf.value.decode('ascii')
        print('found camera ' + device_id)
        if device_id == camera_device_id:
            return driver.OpenQHYCCD(device_id_buf)

    return None


class QHYInterface:
    def __init__(self, config, processing_queue,
                 processing_framebuffer, processing_framebuffer_offsets,
                 processing_stop_signal):
        self._config = config

        self._handle = c_void_p()
        self._driver = None
        self._driver_lock = threading.Lock()

        self._camera_firmware_version = ''
        self._sdk_version = ''

        self._readout_width = 0
        self._readout_height = 0

        if config.filters:
            self._filter = config.filters[0]
        else:
            self._filter = None

        # Streaming frames (aka Live Mode) enables overlapping readout, avoiding
        # dead time between exposures. However, a bug in the QHY600 firmware / SDK
        # >= 20220217 skips the first frame, adding a full exposure dead time at the
        # start of the sequence. If we are only interested in one or two frames it
        # is more efficient to disable streaming and take individual exposures.
        self._stream_frames = True

        # Crop output data to detector coordinates
        self._window_region = [0, 0, 0, 0]
        self._binning = config.binning

        # Image geometry (marking edges of overscan etc)
        self._image_region = [0, 0, 0, 0]

        # Overscan bias pixels
        self._bias_region = [0, 0, 0, 0]

        # Optically masked dark pixels
        self._dark_region = [0, 0, 0, 0]

        self._cooler_condition = threading.Condition()
        self._cooler_mode = CoolerMode.Unknown
        self._cooler_setpoint = config.cooler_setpoint
        self._cooler_temperature = 0
        self._cooler_humidity = 0
        self._cooler_pressure = 0
        self._cooler_pwm = 0

        self._exposure_time = 1
        self._gain = config.gain
        self._offset = config.offset
        self._mode_name = 'UNKNOWN'

        # Limit and number of frames acquired during the next sequence
        # Set to 0 to run continuously
        self._sequence_frame_limit = 0

        # Number of frames acquired this sequence
        self._sequence_frame_count = 0

        # Time that the latest frame in the exposure was started
        self._sequence_exposure_start_time = Time.now()

        # Information for building the output filename
        self._output_directory = pathlib.Path(config.output_path)
        self._output_frame_prefix = config.output_prefix

        # Persistent frame counters
        self._counter_filename = config.expcount_path
        try:
            with open(self._counter_filename, 'r', encoding='ascii') as infile:
                data = json.load(infile)
                self._exposure_count = data['exposure_count']
                self._exposure_count_reference = data['exposure_reference']
        except Exception:
            now = Time.now().strftime('%Y-%m-%d')
            self._exposure_count = 0
            self._exposure_count_reference = now

        # Thread that runs the exposure sequence
        # Initialized by start() method
        self._acquisition_thread = None

        # Signal that the exposure sequence should be terminated
        # at end of the current frame
        self._stop_acquisition = False

        # Subprocess for processing acquired frames
        self._processing_queue = processing_queue
        self._processing_stop_signal = processing_stop_signal

        # A large block of shared memory for sending frame data to the processing workers
        self._processing_framebuffer = processing_framebuffer

        # A queue of memory offsets that are available to write frame data into
        # Offsets are popped from the queue as new frames are written into the
        # frame buffer, and pushed back on as processing is complete
        self._processing_framebuffer_offsets = processing_framebuffer_offsets

        # Camera has been disconnected/powered off or the driver has crashed
        # Detected by way of the temperature/cooler queries returning 0xFFFFFFFF
        self._driver_lost_camera = False

        threading.Thread(target=self.__cooler_thread, daemon=True).start()

    @property
    def is_acquiring(self):
        return self._acquisition_thread is not None and self._acquisition_thread.is_alive()

    @property
    def driver_lost_camera(self):
        return self._driver_lost_camera

    def reset_uvlo(self):
        """Check for and reset if needed the under-voltage lock out flag"""
        with self._driver_lock:
            if int(self._driver.GetQHYCCDParam(self._handle, QHYControl.UVLO_STATUS)) in [2, 3, 9]:
                print('Resetting UVLO flag')
                self._driver.QHYCCDResetFlashULVOError(self._handle)

    def __cooler_thread(self):
        """Polls and updates cooler status"""
        chiller_error = False
        while True:
            with self._driver_lock:
                if self._driver is not None:
                    # Query temperature status
                    temperature = self._driver.GetQHYCCDParam(self._handle, QHYControl.CURTEMP)
                    humidity = self._driver.GetQHYCCDParam(self._handle, QHYControl.HUMIDITY)
                    pressure = self._driver.GetQHYCCDParam(self._handle, QHYControl.PRESSURE)
                    pwm = self._driver.GetQHYCCDParam(self._handle, QHYControl.CURPWM)

                    if temperature == float(0xFFFFFFFF) or pwm == float(0xFFFFFFFF):
                        self._driver_lost_camera = True
                        return

                    self._cooler_temperature = temperature
                    self._cooler_humidity = humidity
                    self._cooler_pressure = pressure
                    self._cooler_pwm = pwm

                    if int(self._driver.GetQHYCCDParam(self._handle, QHYControl.UVLO_STATUS)) in [2, 3, 9]:
                        self._cooler_mode = CoolerMode.UVLOError
                    elif self._cooler_setpoint is None:
                        # Ramp the cooler power down over a few update cycles
                        if self._cooler_pwm > 0:
                            self._cooler_mode = CoolerMode.Warming
                            p = max(0, self._cooler_pwm - self._config.cooler_pwm_step)
                            status = self._driver.SetQHYCCDParam(self._handle, QHYControl.MANUALPWM, c_double(p))
                            if status != QHYStatus.Success:
                                print(f'failed to update cooler PWM control with status {status}')
                        else:
                            self._cooler_mode = CoolerMode.Warm
                    else:
                        temp_delta = abs(self._cooler_temperature - self._cooler_setpoint)
                        if temp_delta > 5:
                            # Ramp the cooler power towards the requested temperature over a few update cycles
                            if self._cooler_temperature > self._cooler_setpoint:
                                self._cooler_mode = CoolerMode.Cooling
                                p = min(255, self._cooler_pwm + self._config.cooler_pwm_step)
                            else:
                                self._cooler_mode = CoolerMode.Warming
                                p = max(0, self._cooler_pwm - self._config.cooler_pwm_step)

                            status = self._driver.SetQHYCCDParam(self._handle, QHYControl.MANUALPWM, c_double(p))
                            if status != QHYStatus.Success:
                                print(f'failed to update cooler PWM control with status {status}')
                        else:
                            self._cooler_mode = CoolerMode.Locked if temp_delta < 0.5 else CoolerMode.Locking

                            target = self._driver.GetQHYCCDParam(self._handle, QHYControl.COOLER)
                            if abs(target - self._cooler_setpoint) > 0.1:
                                # Switch to auto control and/or update new target temperature
                                status = self._driver.SetQHYCCDParam(self._handle, QHYControl.COOLER,
                                                                     c_double(self._cooler_setpoint))
                                if status != QHYStatus.Success:
                                    print(f'failed to set temperature to {self._cooler_setpoint} with status {status}')

            if self._config.chiller_daemon and self._cooler_mode != CoolerMode.Warm:
                try:
                    with self._config.chiller_daemon.connect() as chiller:
                        chiller.notify_camera_cooling_active()
                    chiller_error = False
                except Exception:
                    if not chiller_error:
                        log.error(self._config.log_name, 'Failed to notify chiller daemon of cooling status')
                        chiller_error = True

            with self._cooler_condition:
                self._cooler_condition.wait(self._config.cooler_update_delay)

    def __run_exposure_sequence(self, quiet):
        """Worker thread that acquires frames and their times.
           Tagged frames are pushed to the acquisition queue
           for further processing on another thread"""
        framebuffer_slots = 0
        try:
            with self._driver_lock:
                exp = c_double(max(int(1e6 * self._exposure_time), 1))
                status = self._driver.SetQHYCCDParam(self._handle, QHYControl.EXPOSURE, exp)

            if status != QHYStatus.Success:
                log.error(self._config.log_name, f'Failed to set exposure time ({status})')
                return

            # Prepare the framebuffer offsets
            if not self._processing_framebuffer_offsets.empty():
                log.error(self._config.log_name, 'Frame buffer offsets queue is not empty!')
                return

            offset = 0
            frame_size = 2 * self._readout_width * self._readout_height
            while offset + frame_size <= len(self._processing_framebuffer):
                self._processing_framebuffer_offsets.put(offset)
                offset += frame_size
                framebuffer_slots += 1

            if self._stream_frames:
                # Camera will sometimes return stale data out of the DDR buffer
                # Resetting DDR flag seems to work for clearing the buffer
                with self._driver_lock:
                    status = self._driver.SetQHYCCDParam(self._handle, QHYControl.DDR, c_double(1.0))

                if status != QHYStatus.Success:
                    log.error(self._config.log_name, f'Failed to clear DDR ({status})')
                    return

                with self._driver_lock:
                    status = self._driver.BeginQHYCCDLive(self._handle)

                if status != QHYStatus.Success:
                    log.error(self._config.log_name, f'Failed to start exposures ({status})')
                    return

            pixel_period_ps = c_uint32()
            line_period_ns = c_uint32()
            frame_period_us = c_uint32()
            clocks_per_line = c_uint32()
            lines_per_frame = c_uint32()
            actual_exposure_us = c_uint32()
            is_long_exposure = c_uint8()
            row = c_uint32(0)
            readout_offset_us = c_double()
            width = c_uint32(self._readout_width)
            height = c_uint32(self._readout_height)
            bpp = c_uint32(16)
            channels = c_uint32(1)

            with self._driver_lock:
                self._driver.GetQHYCCDPreciseExposureInfo(self._handle,
                                                          byref(pixel_period_ps),
                                                          byref(line_period_ns),
                                                          byref(frame_period_us),
                                                          byref(clocks_per_line),
                                                          byref(lines_per_frame),
                                                          byref(actual_exposure_us),
                                                          byref(is_long_exposure))

            with self._driver_lock:
                self._driver.GetQHYCCDRollingShutterEndOffset(self._handle, row, byref(readout_offset_us))

            while not self._stop_acquisition and not self._processing_stop_signal.value:
                self._sequence_exposure_start_time = Time.now()
                if not self._stream_frames:
                    # Camera will sometimes return stale data out of the DDR buffer
                    # Resetting DDR flag seems to work for clearing the buffer
                    with self._driver_lock:
                        status = self._driver.SetQHYCCDParam(self._handle, QHYControl.DDR, c_double(1.0))

                    if status != QHYStatus.Success:
                        log.error(self._config.log_name, f'Failed to clear DDR ({status})')
                        break

                    with self._driver_lock:
                        status = self._driver.ExpQHYCCDSingleFrame(self._handle)

                    if status == QHYStatus.Error:
                        log.error(self._config.log_name, f'Failed to start exposure sequence ({status})')
                        break

                framebuffer_offset = self._processing_framebuffer_offsets.get()
                cdata = (c_uint8 * frame_size).from_buffer(self._processing_framebuffer, framebuffer_offset)

                if self._stream_frames:
                    status = QHYStatus.Error
                    while status != QHYStatus.Success:
                        with self._driver_lock:
                            status = self._driver.GetQHYCCDLiveFrame(
                                self._handle, byref(width), byref(height), byref(bpp), byref(channels), cdata)

                        if self._stop_acquisition or self._processing_stop_signal.value:
                            break
                else:
                    with self._driver_lock:
                        status = self._driver.GetQHYCCDSingleFrame(
                            self._handle, byref(width), byref(height), byref(bpp), byref(channels), cdata)

                    # Check for a timed out exposure (all-zero buffer)
                    # The GPS sequence number will never be zero for a real frame
                    seqnum = np.frombuffer(self._processing_framebuffer, dtype=np.dtype('>u4'),
                                           offset=framebuffer_offset, count=1)[0]

                    if status != QHYStatus.Success or seqnum == 0:
                        log.warning(self._config.log_name, f'Exposure failed')
                        self._processing_framebuffer_offsets.put(framebuffer_offset)
                        continue

                if self._stop_acquisition or self._processing_stop_signal.value:
                    # Return unused slot back to the queue to simplify cleanup
                    self._processing_framebuffer_offsets.put(framebuffer_offset)
                    break

                read_end_time = Time.now()

                self._processing_queue.put({
                    'data_offset': framebuffer_offset,
                    'data_width': self._readout_width,
                    'data_height': self._readout_height,
                    'requested_exposure': float(self._exposure_time),
                    'exposure': actual_exposure_us.value / 1e6,
                    'lineperiod': line_period_ns.value / 1e9,
                    'frameperiod': frame_period_us.value / 1e6,
                    'readout_offset': readout_offset_us.value / 1e6,
                    'mode': self._config.mode,
                    'mode_name': self._mode_name,
                    'gain': self._gain,
                    'offset': self._offset,
                    'stream': self._stream_frames,
                    'read_end_time': read_end_time,
                    'filter': self._filter,
                    'sdk_version': self._sdk_version,
                    'firmware_version': self._camera_firmware_version,
                    'cooler_mode': self._cooler_mode,
                    'cooler_temperature': self._cooler_temperature,
                    'cooler_humidity': self._cooler_humidity,
                    'cooler_pressure': self._cooler_pressure,
                    'cooler_pwm': self._cooler_pwm,
                    'cooler_setpoint': self._cooler_setpoint,
                    'window_region': self._window_region,
                    'binning': self._binning,
                    'image_region': self._image_region,
                    'bias_region': self._bias_region,
                    'dark_region': self._dark_region,
                    'exposure_count': self._exposure_count,
                    'exposure_count_reference': self._exposure_count_reference
                })

                self._exposure_count += 1
                self._sequence_frame_count += 1

                # Continue exposure sequence?
                if 0 < self._sequence_frame_limit <= self._sequence_frame_count:
                    self._stop_acquisition = True
        finally:
            if self._stream_frames:
                with self._driver_lock:
                    self._driver.CancelQHYCCDExposingAndReadout(self._handle)
                    self._driver.StopQHYCCDLive(self._handle)

            # Save updated counts to disk
            with open(self._counter_filename, 'w', encoding='ascii') as outfile:
                json.dump({
                    'exposure_count': self._exposure_count,
                    'exposure_reference': self._exposure_count_reference,
                }, outfile)

            # Wait for processing to complete
            for _ in range(framebuffer_slots):
                self._processing_framebuffer_offsets.get()

            if not quiet:
                log.info(self._config.log_name, 'Exposure sequence complete')
            self._stop_acquisition = False

    @Pyro4.expose
    def initialize(self):
        """Connects to the camera driver"""
        print('initializing driver')
        with self._driver_lock:
            # pylint: disable=import-outside-toplevel
            if platform.system() == 'Windows':
                from ctypes import WinDLL
                driver = WinDLL(r'C:\Program Files\QHYCCD\AllInOne\sdk\x64\qhyccd.dll')
            else:
                from ctypes import CDLL
                driver = CDLL('/usr/lib64/libqhyshim.so')
            # pylint: enable=import-outside-toplevel

            driver.OpenQHYCCD.restype = POINTER(c_uint32)
            driver.GetQHYCCDParam.restype = c_double
            handle = None
            initialized = False
            try:
                sdk_version_year = c_uint32()
                sdk_version_month = c_uint32()
                sdk_version_day = c_uint32()
                sdk_version_subday = c_uint32()
                status = driver.GetQHYCCDSDKVersion(
                    byref(sdk_version_year), byref(sdk_version_month),
                    byref(sdk_version_day), byref(sdk_version_subday))
                if status != QHYStatus.Success:
                    print(f'failed to query QHY SDK version with status {status}')
                    return CommandStatus.Failed

                year = sdk_version_year.value
                month = sdk_version_month.value
                day = sdk_version_day.value
                subday = sdk_version_subday.value
                self._sdk_version = f'20{year:02d}{month:02d}{day:02d}_{subday}'

                status = driver.InitQHYCCDResource()
                if status != QHYStatus.Success:
                    print(f'failed to initialize QHY library with status {status}')
                    return CommandStatus.Failed

                # Enumerate cameras to find target
                handle = open_device(driver, self._config.camera_device_id)
                if handle is None:
                    print(f'camera {self._config.camera_device_id} was not found')
                    return CommandStatus.CameraNotFound

                if 'PCIE' in self._config.camera_device_id:
                    # GetQHYCCDFWVersion always returns 201600 for PCIE-connected cameras.
                    fwv = create_string_buffer(4)
                    index = c_uint8(0)
                    status = driver.GetQHYCCDFPGAVersion(handle, index, fwv)
                    if status != QHYStatus.Success:
                        print(f'failed to query FPGA version with status {status}')
                        return CommandStatus.Failed

                    year = fwv.raw[0]
                    month = fwv.raw[1]
                    day = fwv.raw[2]
                    self._camera_firmware_version = f'20{year:02d}{month:02d}{day:02d}'
                else:
                    fwv = create_string_buffer(3)
                    status = driver.GetQHYCCDFWVersion(handle, fwv)
                    fwv = fwv.raw
                    if status == QHYStatus.Success:
                        month = fwv[0] & ~0xf0
                        day = fwv[1]
                        if (fwv[0] >> 4) <= 9:
                            year = (fwv[0] >> 4) + 0x10
                        else:
                            year = fwv[0] >> 4

                        self._camera_firmware_version = f'20{year:02d}{month:02d}{day:02d}'
                    else:
                        print(f'failed to query firmware version with status {status}')
                        return CommandStatus.Failed

                status = driver.SetQHYCCDReadMode(handle, c_uint32(self._config.mode))
                if status != QHYStatus.Success:
                    print(f'failed to set read mode with status {status}')
                    return CommandStatus.Failed

                mode_name = create_string_buffer(128)
                status = driver.GetQHYCCDReadModeName(handle, c_uint32(self._config.mode), mode_name)
                if status != QHYStatus.Success:
                    print(f'failed to query read mode name with status {status}')
                    return CommandStatus.Failed

                self._mode_name = mode_name.value.decode('ascii')

                self._stream_frames = self._config.stream
                status = driver.SetQHYCCDStreamMode(handle, self._config.stream)
                if status != QHYStatus.Success:
                    print(f'failed to set stream mode with status {status}')
                    return CommandStatus.Failed

                status = driver.InitQHYCCD(handle)
                if status != QHYStatus.Success:
                    print(f'failed to initialize camera with status {status}')
                    return CommandStatus.Failed

                chip_width_mm = c_double()
                chip_height_mm = c_double()
                image_width = c_int()
                image_height = c_int()
                pixel_width_um = c_double()
                pixel_height_um = c_double()
                image_bpp = c_int()
                status = driver.GetQHYCCDChipInfo(handle,
                                                  byref(chip_width_mm), byref(chip_height_mm),
                                                  byref(image_width), byref(image_height),
                                                  byref(pixel_width_um), byref(pixel_height_um),
                                                  byref(image_bpp))
                if status != QHYStatus.Success:
                    print(f'failed to query chip info with status {status}')
                    return CommandStatus.Failed

                self._readout_width = image_width.value
                self._readout_height = image_height.value

                # Enable GPS timestamping
                if self._config.use_gpsbox:
                    status = driver.SetQHYCCDParam(handle, QHYControl.GPS, c_double(1))
                    if status != QHYStatus.Success:
                        print(f'failed to set GPS box with status {status}')
                        return CommandStatus.Failed

                # Set gain and offset (bias)
                status = driver.SetQHYCCDParam(handle, QHYControl.GAIN, c_double(self._config.gain))
                if status != QHYStatus.Success:
                    print(f'failed to set default gain with status {status}')
                    return CommandStatus.Failed

                status = driver.SetQHYCCDParam(handle, QHYControl.OFFSET, c_double(self._config.offset))
                if status != QHYStatus.Success:
                    print(f'failed to set default offset with status {status}')
                    return CommandStatus.Failed

                # USBTRAFFIC changes the HBLANK behaviour, which impacts the readout timing characteristics.
                # These timing parameters are currently hardcoded in the frame header creation
                # and must be recalibrated if this changes!
                status = driver.SetQHYCCDParam(handle, QHYControl.USBTRAFFIC, c_double(0))
                if status != QHYStatus.Success:
                    print(f'failed to set usbtraffic with status {status}')
                    return CommandStatus.Failed

                status = driver.SetQHYCCDResolution(handle, 0, 0, image_width, image_height)
                if status != QHYStatus.Success:
                    print(f'failed to set readout region with status {status}')
                    return CommandStatus.Failed

                status = driver.SetQHYCCDBitsMode(handle, 16)
                if status != QHYStatus.Success:
                    print(f'failed to set 16bit readout with status {status}')
                    return CommandStatus.Failed

                status = driver.SetQHYCCDSingleFrameTimeOut(handle, 10000)
                if status != QHYStatus.Success:
                    print(f'failed to set single-frame time out with status {status}')
                    return CommandStatus.Failed

                # Regions are 0-indexed x1,x2,y1,2
                # These are converted to 1-indexed when writing fits headers
                self._window_region = [
                    0,
                    image_width.value - 1,
                    0,
                    image_height.value - 1
                ]

                # HACK: Some SDK versions return bogus effective areas,
                # so hardcode regions based on sensor specifications
                self._image_region = [
                    24,
                    self._window_region[1],
                    1 if self._config.use_gpsbox else 0,
                    6387
                ]

                self._bias_region = [
                    self._window_region[0],
                    self._window_region[1],
                    6390,
                    self._window_region[3]
                ]

                self._dark_region = [
                    self._window_region[0],
                    21,
                    self._window_region[2],
                    6387
                ]

                self._cooler_temperature = driver.GetQHYCCDParam(handle, QHYControl.CURTEMP)
                self._cooler_humidity = driver.GetQHYCCDParam(handle, QHYControl.HUMIDITY)
                self._cooler_pressure = driver.GetQHYCCDParam(handle, QHYControl.PRESSURE)
                self._cooler_pwm = driver.GetQHYCCDParam(handle, QHYControl.CURPWM)

                self._driver = driver
                self._handle = handle
                initialized = True
                print(f'camera {self._config.camera_device_id} initialized')

                if len(self._config.filters) > 1:
                    # Filter wheel takes ~16 seconds to home after power-on
                    # IsQHYCCDCFWPlugged returns false until this is complete
                    # Allow up to 20 seconds before giving up.
                    attempts = 0
                    while True:
                        attempts += 1
                        status = self._driver.IsQHYCCDCFWPlugged(handle)
                        if status == QHYStatus.Success:
                            break

                        if attempts == 20:
                            print('filter wheel is not plugged in')
                            return CommandStatus.Failed

                        time.sleep(1)

                    filter_index = c_uint8()
                    status = self._driver.GetQHYCCDCFWStatus(handle, byref(filter_index))
                    if status != QHYStatus.Success:
                        print(f'failed to query active filter with status {status}')
                        return CommandStatus.Failed

                    self._filter = self._config.filters[filter_index.value - ord('0')]
                return CommandStatus.Succeeded
            except Exception as e:
                print(e)
                return CommandStatus.Failed
            finally:
                # Clean up on failure
                if not initialized:
                    if driver is not None and handle is not None:
                        driver.CloseQHYCCD(handle)

                    log.error(self._config.log_name, 'Failed to initialize camera')
                    driver.ReleaseQHYCCDResource()
                else:
                    log.info(self._config.log_name, 'Initialized camera')
                    with self._cooler_condition:
                        self._cooler_condition.notify()


    def set_target_temperature(self, temperature, quiet):
        """Set the target camera temperature"""
        if temperature is not None and (temperature < -20 or temperature > 30):
            return CommandStatus.TemperatureOutsideLimits

        self._cooler_setpoint = temperature
        with self._cooler_condition:
            self._cooler_condition.notify()

        if not quiet:
            log.info(self._config.log_name, f'Target temperature set to {temperature}')

        return CommandStatus.Succeeded

    def set_gain(self, gain, quiet):
        """Set the camera gain"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        with self._driver_lock:
            self._gain = gain
            status = self._driver.SetQHYCCDParam(self._handle, QHYControl.GAIN, c_double(self._gain))

            if status != QHYStatus.Success:
                print(f'failed to set gain with status {status}')
                return CommandStatus.Failed

        if not quiet:
            log.info(self._config.log_name, f'Gain set to {gain}')

        return CommandStatus.Succeeded

    def set_offset(self, offset, quiet):
        """Set the camera bias level"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        with self._driver_lock:
            self._offset = offset
            status = self._driver.SetQHYCCDParam(self._handle, QHYControl.OFFSET, c_double(self._offset))
            if status != QHYStatus.Success:
                print(f'failed to set offset with status {status}')
                return CommandStatus.Failed

        if not quiet:
            log.info(self._config.log_name, f'Offset set to {offset}')

        return CommandStatus.Succeeded

    def set_exposure(self, exposure, quiet):
        """Set the camera exposure time"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        self._exposure_time = exposure
        if not quiet:
            log.info(self._config.log_name, f'Exposure time set to {exposure:.3f}s')

        return CommandStatus.Succeeded

    def set_window(self, window, quiet):
        """Set the camera crop window"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        if window is None:
            self._window_region = [0, self._readout_width - 1, 0, self._readout_height - 1]

        elif len(window) == 4:
            if window[0] < 1 or window[0] > self._readout_width:
                return CommandStatus.WindowOutsideSensor
            if window[1] < window[0] or window[1] > self._readout_width:
                return CommandStatus.WindowOutsideSensor
            if window[2] < 1 or window[2] > self._readout_height:
                return CommandStatus.WindowOutsideSensor
            if window[3] < window[2] or window[3] > self._readout_height:
                return CommandStatus.WindowOutsideSensor

            # Convert from 1-indexed to 0-indexed
            self._window_region = [x - 1 for x in window]

        else:
            return CommandStatus.Failed

    def set_binning(self, binning, quiet):
        """Set the camera binning"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        if binning is None:
            binning = self._config.binning

        if not isinstance(binning, int) or binning < 1:
            return CommandStatus.Failed

        self._binning = binning

        if not quiet:
            log.info(self._config.log_name, f'Binning set to {binning}')

        return CommandStatus.Succeeded

    def set_filter(self, filter_name, quiet):
        """Set the filter wheel active filter"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        # Invalid filter or no filter wheel installed
        if len(self._config.filters) < 2 or filter_name not in self._config.filters:
            return CommandStatus.Failed

        with self._driver_lock:
            filter_index = ord('0') + self._config.filters.index(filter_name)
            status = self._driver.SendOrder2QHYCCDCFW(self._handle, byref(c_uint8(filter_index)), 1)
            if status != QHYStatus.Success:
                print(f'failed to set filter with status {status}')
                return CommandStatus.Failed

            self._filter = filter_name

        if not quiet:
            log.info(self._config.log_name, f'Filter set to {filter_name}')

        return CommandStatus.Succeeded

    def set_frame_streaming(self, stream, quiet):
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        if self._stream_frames == stream:
            return CommandStatus.Succeeded

        with self._driver_lock:
            status = self._driver.SetQHYCCDStreamMode(self._handle, 1 if stream else 0)
            if status != QHYStatus.Success:
                print(f'failed to set stream mode with status {status}')
                return CommandStatus.Failed

            self._stream_frames = stream

            status = self._driver.InitQHYCCD(self._handle)
            if status != QHYStatus.Success:
                print(f'failed to initialize camera with status {status}')
                return CommandStatus.Failed

            status = self._driver.SetQHYCCDResolution(self._handle, 0, 0, self._readout_width, self._readout_height)
            if status != QHYStatus.Success:
                print(f'failed to set readout region with status {status}')
                return CommandStatus.Failed

            status = self._driver.SetQHYCCDBitsMode(self._handle, 16)
            if status != QHYStatus.Success:
                print(f'failed to set 16bit readout with status {status}')
                return CommandStatus.Failed

            if self._config.use_gpsbox:
                status = self._driver.SetQHYCCDParam(self._handle, QHYControl.GPS, c_double(1))
                if status != QHYStatus.Success:
                    print(f'failed to set GPS box with status {status}')
                    return CommandStatus.Failed

            if not quiet:
                log.info(self._config.log_name, f'Streaming set to {stream}')

            return CommandStatus.Succeeded

    @Pyro4.expose
    def start_sequence(self, count, quiet):
        """Starts an exposure sequence with a set number of frames, or 0 to run until stopped"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        self._sequence_frame_limit = count
        self._sequence_frame_count = 0
        self._stop_acquisition = False
        self._processing_stop_signal.value = False

        self._acquisition_thread = threading.Thread(
            target=self.__run_exposure_sequence,
            args=(quiet,), daemon=True)
        self._acquisition_thread.start()

        if not quiet:
            count_msg = 'until stopped'
            if count == 1:
                count_msg = '1 frame'
            elif count > 1:
                count_msg = f'{count} frames'

            log.info(self._config.log_name, f'Starting exposure sequence ({count_msg})')

        return CommandStatus.Succeeded

    @Pyro4.expose
    def stop_sequence(self, quiet):
        """Stops any active exposure sequence"""
        if not self.is_acquiring or self._stop_acquisition:
            return CommandStatus.CameraNotAcquiring

        if not quiet:
            log.info(self._config.log_name, 'Aborting exposure sequence')

        self._sequence_frame_count = 0
        self._stop_acquisition = True

        return CommandStatus.Succeeded

    def report_status(self):
        """Returns a dictionary containing the current camera state"""
        # Estimate the current frame progress based on the time delta
        exposure_progress = 0
        sequence_frame_count = self._sequence_frame_count
        state = CameraStatus.Idle

        if self.is_acquiring:
            state = CameraStatus.Acquiring
            if self._stop_acquisition:
                state = CameraStatus.Aborting
            else:
                if self._sequence_exposure_start_time is not None:
                    exposure_progress = (Time.now() - self._sequence_exposure_start_time).to(u.s).value
                    if exposure_progress >= self._exposure_time:
                        state = CameraStatus.Reading

        return {
            'state': state,
            'cooler_mode': self._cooler_mode,
            'cooler_temperature': self._cooler_temperature,
            'cooler_humidity': self._cooler_humidity,
            'cooler_pressure': self._cooler_pressure,
            'cooler_pwm': round(self._cooler_pwm / 2.55),  # byte to percentage
            'cooler_setpoint': self._cooler_setpoint,
            'temperature_locked': self._cooler_mode == CoolerMode.Locked,  # used by opsd
            'exposure_time': self._exposure_time,
            'exposure_progress': exposure_progress,
            'window': self._window_region,
            'binning': self._binning,
            'sequence_frame_limit': self._sequence_frame_limit,
            'sequence_frame_count': sequence_frame_count,
            'filter': self._filter,
            'stream': self._stream_frames
        }

    def shutdown(self):
        """Disconnects from the camera driver"""
        # Complete the current exposure
        if self._acquisition_thread is not None:
            with self._driver_lock:
                self._driver.CancelQHYCCDExposingAndReadout(self._handle)
            print('shutdown: waiting for acquisition to complete')
            self._stop_acquisition = True
            self._acquisition_thread.join()

        with self._driver_lock:
            print('shutdown: disconnecting driver')
            self._driver.CloseQHYCCD(self._handle)
            self._driver.ReleaseQHYCCDResource()
            self._driver = None

        log.info(self._config.log_name, 'Shutdown camera')
        return CommandStatus.Succeeded


def qhy_process(camd_pipe, config,
                processing_queue, processing_framebuffer, processing_framebuffer_offsets,
                stop_signal):
    cam = QHYInterface(config, processing_queue, processing_framebuffer, processing_framebuffer_offsets, stop_signal)
    ret = cam.initialize()

    # Clear any UVLO errors on first connection
    if ret == CommandStatus.Succeeded:
        cam.reset_uvlo()

    camd_pipe.send(ret)
    if ret != CommandStatus.Succeeded:
        return

    try:
        while True:
            if camd_pipe.poll(timeout=1):
                c = camd_pipe.recv()
                command = c['command']
                args = c['args']

                if command == 'temperature':
                    camd_pipe.send(cam.set_target_temperature(args['temperature'], args['quiet']))
                elif command == 'stream':
                    camd_pipe.send(cam.set_frame_streaming(args['stream'], args['quiet']))
                elif command == 'gain':
                    camd_pipe.send(cam.set_gain(args['gain'], args['quiet']))
                elif command == 'offset':
                    camd_pipe.send(cam.set_offset(args['offset'], args['quiet']))
                elif command == 'exposure':
                    camd_pipe.send(cam.set_exposure(args['exposure'], args['quiet']))
                elif command == 'window':
                    camd_pipe.send(cam.set_window(args['window'], args['quiet']))
                elif command == 'binning':
                    camd_pipe.send(cam.set_binning(args['binning'], args['quiet']))
                elif command == 'start':
                    camd_pipe.send(cam.start_sequence(args['count'], args['quiet']))
                elif command == 'stop':
                    camd_pipe.send(cam.stop_sequence(args['quiet']))
                elif command == 'filter':
                    camd_pipe.send(cam.set_filter(args['filter_name'], args['quiet']))
                elif command == 'status':
                    camd_pipe.send(cam.report_status())
                elif command == 'shutdown':
                    break
                else:
                    print(f'unhandled command: {command}')
                    camd_pipe.send(CommandStatus.Failed)

                if cam.driver_lost_camera:
                    log.error(config.log_name, 'camera has disappeared')
                    break

    except Exception:
        traceback.print_exc(file=sys.stdout)
        camd_pipe.send(CommandStatus.Failed)

    camd_pipe.close()
    cam.shutdown()
