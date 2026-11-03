# PlatformIO post-script: flash over USB-C without pressing BOOT0.
#
# Before a DFU upload, opens the board's USB CDC port at 1200 baud and closes
# it. The firmware (src/dfu_bootloader.cpp) reboots into the STM32 ROM DFU
# bootloader; we then wait (bounded) for the DFU device to appear.
#
# If the board is already in DFU mode (no CDC port found), the touch is
# skipped and dfu-util flashes as usual.

Import("env")  # noqa: F821

import os
import subprocess
import time

STM_VID = 0x0483
STM_CDC_PID = 0x5740
DFU_ID = "0483:df11"
DFU_WAIT_S = 10


def _find_cdc_port():
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        if p.vid == STM_VID and p.pid == STM_CDC_PID:
            return p.device
    return None


def _dfu_present(env):
    dfu_util = os.path.join(
        env.PioPlatform().get_package_dir("tool-dfuutil") or "", "bin", "dfu-util")
    try:
        out = subprocess.run([dfu_util, "-l"], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    return DFU_ID in out.lower()


def _touch_1200(source, target, env):
    import serial

    if _dfu_present(env):
        print("Board already in DFU mode")
        return

    port = env.subst("$UPLOAD_PORT") or _find_cdc_port()
    if not port:
        print("No MARS CDC port and no DFU device found. Is the board plugged in?")
        return

    print("Sending 1200 baud touch to %s to enter DFU bootloader" % port)
    try:
        s = serial.Serial(port, 1200)
        s.dtr = False
        s.close()
    except (serial.SerialException, OSError) as e:
        # The board may drop off the bus mid-close; that's expected.
        print("Touch: %s" % e)

    deadline = time.time() + DFU_WAIT_S
    while time.time() < deadline:
        time.sleep(0.5)
        if _dfu_present(env):
            print("Board entered DFU mode")
            return

    print(
        "\n*** Board did not enter DFU mode after the 1200 baud touch. ***\n"
        "The firmware on the board probably doesn't include src/dfu_bootloader.cpp\n"
        "yet (or has crashed). Hold BOOT0 high, reset the board, and upload again.\n"
    )


if env.subst("$UPLOAD_PROTOCOL") == "dfu":
    env.AddPreAction("upload", _touch_1200)
