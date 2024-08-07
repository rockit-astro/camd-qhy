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

"""client command input handlers"""

import Pyro4
from rockit.common import print
from .config import Config
from .constants import CommandStatus, CameraStatus, CoolerMode


def run_client_command(config_path, usage_prefix, args):
    """Prints the message associated with a status code and returns the code"""
    config = Config(config_path)
    commands = {
        'temperature': set_temperature,
        'exposure': set_exposure,
        'window': set_window,
        'bin': set_binning,
        'gain': set_gain,
        'offset': set_offset,
        'stream': set_streaming,
        'filter': set_filter,
        'status': status,
        'start': start,
        'stop': stop,
        'init': initialize,
        'kill': shutdown,
    }

    if len(args) == 0 or (args[0] not in commands and args[0] != 'completion'):
        return print_usage(usage_prefix)

    if args[0] == 'completion':
        if 'filter' in args[-2:]:
            print(' '.join(config.filters))
        elif 'start' in args[-2:]:
            print('continuous')
        elif 'stream' in args[-2:]:
            print('enable disable')
        elif 'temperature' in args[-2:]:
            print('warm')
        elif 'window' in args[-2:]:
            print('default')
        elif 'bin' in args[-3:-1]:
            print('add mean')
        elif len(args) < 3:
            print(' '.join(commands))
        return 0

    try:
        ret = commands[args[0]](config, usage_prefix, args[1:])
    except KeyboardInterrupt:
        # ctrl-c terminates the running command
        ret = stop(config, args)

        # Report successful stop
        if ret == 0:
            ret = -100
    except Pyro4.errors.CommunicationError:
        ret = -101

    # Print message associated with error codes
    if ret not in [-1, 0]:
        print(CommandStatus.message(ret))

    return ret


def status(config, *_):
    """Reports the current camera status"""
    with config.daemon.connect() as camd:
        data = camd.report_status()

    state_desc = CameraStatus.label(data['state'], True)
    if data['state'] == CameraStatus.Acquiring:
        state_desc += f' ([b]{data["exposure_progress"]:.1f} / {data["exposure_time"]:.1f}s[/b])'

    # Camera is disabled
    print(f'   Camera is {state_desc}')
    if data['state'] != CameraStatus.Disabled:
        if data['state'] > CameraStatus.Idle:
            if data['sequence_frame_limit'] > 0:
                print(f'   Acquiring frame [b]{data["sequence_frame_count"] + 1} / {data["sequence_frame_limit"]}[/b]')
            else:
                print(f'   Acquiring [b]UNTIL STOPPED[/b]')

        print(f'   Temperature is [b]{data["cooler_temperature"]:.0f}\u00B0C[/b]' +
              f' ([b]{data["cooler_pwm"]:.0f}%[/b] power, {CoolerMode.label(data["cooler_mode"], True)})')

        if data['cooler_setpoint'] is not None:
            print(f'   Temperature set point is [b]{data["cooler_setpoint"]:.0f}\u00B0C[/b]')

        print(f'   Frame streaming is [b]{"ENABLED" if data["stream"] else "DISABLED"}[/b]')
        print(f'   Exposure time is [b]{data["exposure_time"]:.3f} s[/b]')

        w = [x + 1 for x in data['window']]
        print(f'   Output window is [b]\\[{w[0]}:{w[1]},{w[2]}:{w[3]}][/b]')
        print(f'   Binning is [b]{data["binning"]}x{data["binning"]}[/b] ([b]{data["binning_method"]}[/b])')
        if data['filter']:
            print(f'   Filter is [b]{data["filter"]}[/b]')
    return 0


def set_temperature(config, usage_prefix, args):
    """Set the camera temperature"""
    if len(args) == 1:
        if args[0] == 'warm':
            temp = None
        else:
            temp = int(args[0])
        with config.daemon.connect() as camd:
            return camd.set_target_temperature(temp)
    print(f'usage: {usage_prefix} temperature <degrees>')
    return -1


def set_exposure(config, usage_prefix, args):
    """Set the camera exposure time"""
    if len(args) == 1:
        exposure = float(args[0])
        with config.daemon.connect() as camd:
            return camd.set_exposure(exposure)
    print(f'usage: {usage_prefix} exposure <seconds>')
    return -1


def set_gain(config, usage_prefix, args):
    """Set the camera exposure time"""
    if len(args) == 1:
        gain = int(args[0])
        with config.daemon.connect() as camd:
            return camd.set_gain(gain)
    print(f'usage: {usage_prefix} gain <value>')
    return -1


def set_window(config, usage_prefix, args):
    """Set the camera readout window"""
    window = None
    if len(args) == 4:
        window = [
            int(args[0]),
            int(args[1]),
            int(args[2]),
            int(args[3])
        ]

    if window or (len(args) == 1 and args[0] == 'default'):
        with config.daemon.connect() as camd:
            return camd.set_window(window)

    print(f'usage: {usage_prefix} window <x1 x2 y1 y2|default>')
    return -1


def set_binning(config, usage_prefix, args):
    """Set the camera binning"""
    if len(args) == 1 and args[0] == 'default':
        binning = method = None
    elif len(args) == 2 and args[1] in ['sum', 'mean']:
        try:
            binning = int(args[0])
        except ValueError:
            print(f'usage: {usage_prefix} bin <pixels> <sum|mean>')
            return -1
        method = args[1]
    else:
        print(f'usage: {usage_prefix} bin <pixels> <sum|mean>')
        return -1

    with config.daemon.connect() as camd:
        return camd.set_binning(binning, method)



def set_streaming(config, usage_prefix, args):
    """Set the camera streaming mode"""
    if len(args) == 1 and (args[0] == 'enable' or args[0] == 'disable'):
        enabled = args[0] == 'enable'
        with config.daemon.connect() as camd:
            return camd.set_frame_streaming(enabled)
    print(f'usage: {usage_prefix} stream <enable|disable>')
    return -1


def set_filter(config, usage_prefix, args):
    """Set the active filter"""
    if len(args) == 1 and (args[0] in config.filters):
        with config.daemon.connect() as camd:
            return camd.set_filter(args[0])
    print(f'usage: {usage_prefix} filter <{"|".join(config.filters)}>')
    return -1


def set_offset(config, usage_prefix, args):
    """Set the camera exposure time"""
    if len(args) == 1:
        offset = int(args[0])
        with config.daemon.connect() as camd:
            return camd.set_offset(offset)
    print(f'usage: {usage_prefix} offset <value>')
    return -1


def start(config, usage_prefix, args):
    """Starts an exposure sequence"""
    if len(args) == 1:
        try:
            count = 0 if args[0] == 'continuous' else int(args[0])
            if args[0] == 'continuous' or count > 0:
                with config.daemon.connect() as camd:
                    return camd.start_sequence(count)
        except Exception:
            print('error: invalid exposure count:', args[0])
            return -1
    print(f'usage: {usage_prefix} start <continuous|(count)>')
    return -1


def stop(config, *_):
    """Stops any active camera exposures"""
    with config.daemon.connect() as camd:
        return camd.stop_sequence()


def initialize(config, *_):
    """Enables the camera driver"""
    # Initialization can take more than 5 sec, so bump timeout to 10.
    # Filter wheel takes ~16 seconds to home after power-on, so allow an extra 20 to compensate
    timeout = 30 if len(config.filters) > 1 else 10
    with config.daemon.connect(timeout=timeout) as camd:
        return camd.initialize()


def shutdown(config, *_):
    """Disables the camera drivers"""
    with config.daemon.connect() as camd:
        return camd.shutdown()


def print_usage(usage_prefix):
    """Prints the utility help"""
    print(f'usage: {usage_prefix} <command> \\[<args>]')
    print()
    print('general commands:')
    print('   status       print a human-readable summary of the camera status')
    print('   exposure     set exposure time in seconds')
    print('   start        start an exposure sequence')
    print('   window       set image geometry')
    print('   bin          set image binning')
    print('   gain         set cmos gain parameter')
    print('   offset       set cmos bias parameter')
    print('   stream       switch between single-exposure and live mode')
    print()
    print('engineering commands:')
    print('   init         connect to and initialize the camera')
    print('   temperature  set target temperature and enable cooling')
    print('   kill         disconnect from the camera')
    print()

    return 0

