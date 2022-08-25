#
# This file is part of qhy-camd.
#
# qhy-camd is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# qhy-camd is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with qhy-camd.  If not, see <http://www.gnu.org/licenses/>.

"""Helper process for preparing and saving fits images"""

# pylint: disable=too-many-arguments

from ctypes import c_uint8, c_uint16, c_uint32, Structure
import os.path
import shutil
from astropy.io import fits
from astropy.time import Time
import astropy.units as u
from warwick.observatory.common import daemons, log
from .constants import CoolerMode


class GPSData(Structure):
    _pack_ = 1
    _fields_ = [
        ("SequenceNumber", c_uint32.__ctype_be__),
        ("unused1", c_uint8),
        ("ImageWidth", c_uint16.__ctype_be__),
        ("ImageHeight", c_uint16.__ctype_be__),
        ("_Latitude", c_uint32.__ctype_be__),
        ("_Longitude", c_uint32.__ctype_be__),
        ("StartFlag", c_uint8),
        ("StartSeconds", c_uint32.__ctype_be__),
        ("StartCounts", 3 * c_uint8),
        ("EndFlag", c_uint8),
        ("EndSeconds", c_uint32.__ctype_be__),
        ("EndCounts", 3 * c_uint8),
        ("NowFlag", c_uint8),
        ("NowSeconds", c_uint32.__ctype_be__),
        ("NowCounts", 3 * c_uint8),
        ("_PPSDelta", 3 * c_uint8),
    ]

    @classmethod
    def create_timestamp(cls, seconds, count_bytes):
        # Timestamps are encoded as the number of seconds since 2450000.5 JD plus number of
        # 10MHz (0.1us) clock cycles (not microseconds!).
        counts = int.from_bytes(count_bytes, byteorder='big', signed=False)
        return Time((seconds + counts / 1e7) / (3600 * 24) + 2450000.5, format='jd').isot

    @classmethod
    def create_status(cls, flag):
        return [
            'OFFLINE',
            'SEARCHING',
            'LOCKING',
            'LOCKED'
        ][(flag // 16) % 4]

    @property
    def Latitude(self):
        # Latitude is encoded as 10 digits
        # First digit is a sign indicator (1 means negative, 0 positive)
        # Next two digits provide the degree component
        # Remaining seven digits provide fractional component as decimal minutes
        minutes = (self._Latitude % 10000000) / 100000
        degrees = (self._Latitude // 10000000) % 100
        sign = -1 if self._Latitude > 1000000000 else 1
        return sign * (degrees + minutes / 60)

    @property
    def Longitude(self):
        # Longitude is encoded as 10 digits
        # First digit is a sign indicator (1 means negative, 0 positive)
        # Next three digits provide the degree component
        # Remaining six digits provide fractional component as decimal minutes
        minutes = (self._Longitude % 1000000) / 10000
        degrees = (self._Longitude // 1000000) % 100
        sign = -1 if self._Longitude > 1000000000 else 1
        return sign * (degrees + minutes / 60)

    @property
    def PPSDelta(self):
        return int.from_bytes(self._PPSDelta, byteorder='big', signed=False)


def output_process(process_queue, stop_signal, camera_id, camera_device_id, use_gpsbox,
                   filter_name, header_card_capacity, output_path, log_name,
                   pipeline_daemon_name, pipeline_handover_timeout, software_version):
    """
    Helper process to save frames to disk.
    This uses a process (rather than a thread) to avoid the GIL bottlenecking throughput,
    and multiple worker processes allow frames to be handled in parallel.
    """
    pipeline_daemon = getattr(daemons, pipeline_daemon_name)
    while True:
        frame = process_queue.get()

        # Estimate frame end time based on when we finished reading out
        # line period is given in nanoseconds
        # TODO: Calibrate from GPS box offsets
        # HACK: on SDK version 22.02.17 frames over USB appear to be delayed by an extra frame period
        # TODO: Test if this is true for PCIE
        end_offset = -frame['lineperiod'] * 6422 / 1e9 - frame['frameperiod']
        start_offset = end_offset - frame['exposure']
        end_time = (frame['read_end_time'] + end_offset * u.s).strftime('%Y-%m-%dT%H:%M:%S.%f')
        start_time = (frame['read_end_time'] + start_offset * u.s).strftime('%Y-%m-%dT%H:%M:%S.%f')
        date_header = [
            ('DATE-OBS', start_time, '[utc] estimated row 0 exposure start time'),
            ('DATE-END', end_time, '[utc] estimated row 0 exposure end time'),
            ('TIME-SRC', 'NTP', 'DATE-OBS is estimated from NTP-synced PC clock'),
        ]
        gps_header = []

        if use_gpsbox:
            # Parse timestamps out of the first row of pixel data
            # The GPS box and data protocol seem have been designed for a global shutter camera,
            # so the start/end/now timestamps here actually record the rising and falling edge
            # of the 0.4us long VSYNC signal and the start/end values are meaningless.
            # SDK versions >= 22.02.17 add precision timing APIs to calculate the relative offsets
            # to the first image row.

            gps = GPSData.from_address(frame['data'].ctypes.data)
            vsync_timestamp = GPSData.create_timestamp(gps.NowSeconds, gps.NowCounts)
            vsync_status = GPSData.create_status(gps.NowFlag)

            if vsync_status == 'LOCKED':
                end_seconds = gps.NowSeconds + frame['readout_offset'] / 1e6
                start_time = GPSData.create_timestamp(end_seconds - frame['exposure'], gps.NowCounts)
                end_time = GPSData.create_timestamp(end_seconds, gps.NowCounts)
                date_header = [
                    ('DATE-OBS', start_time, '[utc] row 0 exposure start time'),
                    ('DATE-END', end_time, '[utc] row 0 exposure end time'),
                    ('TIME-SRC', 'GPS', 'DATE-OBS is from a GPS measured HSYNC signal'),
                ]

            gps_header = [
                (None, None, None),
                ('COMMENT', ' ---             GPS INFORMATION             --- ', ''),
                ('GPS-SEQN', gps.SequenceNumber, 'exposure sequence number'),
                ('GPS-LAT', gps.Latitude, '[deg] latitude reported by the camera GPS'),
                ('GPS-LON', gps.Longitude, '[deg] longitude reported by the camera GPS'),
                ('GPS-VSYN', vsync_timestamp, 'gps timestamp of vsync falling edge'),
                ('GPS-VSTA', vsync_status, 'gps status for the GPS-VSYNC timestamp'),
                ('GPS-PPSD', gps.PPSDelta, 'number of oscillator counts between PPS pulses')
            ]

            # Enabling the GPS box overwrites the first 5 rows of image data
            frame['image_y1'] += 5

        if frame['cooler_setpoint'] is not None:
            setpoint_header = ('TEMP-SET', frame['cooler_setpoint'], '[deg c] cmos temperature set point')
        else:
            setpoint_header = ('COMMENT', ' TEMP-SET not available', '')

        header = [
            (None, None, None),
            ('COMMENT', ' ---                DATE/TIME                --- ', ''),
        ] + date_header + [
            ('EXPTIME', round(frame['exposure'], 3), '[s] actual exposure length'),
            ('EXPRQSTD', round(frame['requested_exposure'], 3), '[s] requested exposure length'),
            ('EXPCADNC', round(frame['frameperiod'], 3), '[s] exposure cadence'),
            ('ROWDELTA', round(frame['lineperiod'] * 1e6, 3), '[us] rolling shutter row period'),
            ('PC-RDEND', frame['read_end_time'].strftime('%Y-%m-%dT%H:%M:%S.%f'),
             '[utc] local PC time when readout completed'),
            (None, None, None),
            ('COMMENT', ' ---           CAMERA INFORMATION            --- ', ''),
            ('CAMSWVER', software_version, 'camera server software version'),
            ('SDKVER', frame['sdk_version'], 'QHY SDK version'),
            ('FWVER', frame['firmware_version'], 'camera firmware version'),
            ('CAMID', camera_id, 'camera identifier'),
            ('CAMERA', camera_device_id, 'camera model and serial number'),
            ('FILTER', filter_name, 'filter installed in camera path'),
            ('CAM-MODE', frame['mode'], 'cmos read mode ({})'.format(frame['mode_name'])),
            ('CAM-TFER', 'STREAM' if frame['stream'] else 'SINGLE', 'frame transfer mode'),
            ('CAM-GAIN', frame['gain'], 'cmos gain setting'),
            ('CAM-OFST', frame['offset'], 'cmos offset setting'),
            ('CAM-TEMP', round(frame['cooler_temperature'], 2),
             '[deg c] cmos temperature at end of exposure'),
            ('TEMP-MOD', CoolerMode.label(frame['cooler_mode']), 'temperature control mode'),
            ('TEMP-PWR', round(frame['cooler_pwm'] / 2.55), '[%] cooler power'),
            setpoint_header,
            ('TEMP-LCK', frame['cooler_mode'] == CoolerMode.Locked, 'cmos temperature is locked to set point'),
            ('CAM-XBIN', 1, '[px] x binning'),
            ('CAM-YBIN', 1, '[px] y binning'),
            ('CAM-WIND', '[{}:{},{}:{}]'.format(
                frame['win_x'], frame['win_x'] + frame['win_width'] - 1,
                frame['win_y'], frame['win_y'] + frame['win_height'] - 1),
             '[x1:x2,y1:y2] readout region (detector coords)'),
            ('IMAG-RGN', '[{}:{},{}:{}]'.format(
                frame['image_x1'], frame['image_x2'],
                frame['image_y1'], frame['image_y2']),
             '[x1:x2,y1:y2] image region (image coords)'),
            # TODO: These will need to change if windowing is implemented!
            ('BIAS-RGN', '[{}:{},{}:{}]'.format(
                1, 9600,
                6391, 6422),
             '[x1:x2,y1:y2] overscan region (image coords)'),
            ('DARK-RGN', '[{}:{},{}:{}]'.format(
                1, 22,
                1, 6388),
             '[x1:x2,y1:y2] masked dark region (image coords)'),
            ('EXPCNT', frame['exposure_count'], 'running exposure count since EXPCREF'),
            ('EXPCREF', frame['exposure_count_reference'], 'date the exposure counter was reset'),
        ] + gps_header

        hdu = fits.PrimaryHDU(frame['data'])

        # Using Card and append() to force comment cards to be placed inline
        for h in header:
            hdu.header.append(fits.Card(h[0], h[1], h[2]), end=True)

        # Pad with sufficient blank cards that pipelined won't need to allocate extra header blocks
        padding = max(0, header_card_capacity - len(hdu.header) - 1)
        for _ in range(padding):
            hdu.header.append(fits.Card(None, None, None), end=True)

        # Save errors shouldn't interfere with preview updates, so we use a separate try/catch
        try:
            filename = '{}-{:08d}.fits'.format(camera_id, frame['exposure_count'])
            path = os.path.join(output_path, filename)

            # Simulate an atomic write by writing to a temporary file then renaming
            hdu.writeto(path + '.tmp', overwrite=True)
            shutil.move(path + '.tmp', path)
            print('Saving temporary frame: ' + filename)

        except Exception as e:
            stop_signal.value = True
            log.error(log_name, 'Failed to save temporary frame (' + str(e) + ')')

        # Hand frame over to the pipeline
        # This may block if the pipeline is busy
        try:
            with pipeline_daemon.connect(pipeline_handover_timeout) as pipeline:
                pipeline.notify_frame(camera_id, filename)
        except Exception as e:
            stop_signal.value = True
            log.error(log_name, 'Failed to hand frame to pipeline (' + str(e) + ')')
