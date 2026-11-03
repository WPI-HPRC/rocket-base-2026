# Runs dfu-util (argv[1:]) and treats a failure after the leave request as
# success, as long as dfu-util already reported the download completed.
# Used by scripts/dfu_touch.py.

import subprocess
import sys

proc = subprocess.Popen(sys.argv[1:], stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, text=True, bufsize=1)
downloaded = False
for line in proc.stdout:
    sys.stdout.write(line)
    sys.stdout.flush()
    if "File downloaded successfully" in line:
        downloaded = True
rc = proc.wait()

if rc != 0 and downloaded:
    print("dfu-util lost the device after :leave. That's expected; the flash "
          "completed and the board is rebooting into the new firmware.")
    rc = 0
sys.exit(rc)
