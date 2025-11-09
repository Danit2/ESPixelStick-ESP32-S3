#!/usr/bin/env python3
"""
Modernized upload.py wrapper for ESPixelStick / ESP32-S3
Compatible with esptool >= 4.x
"""

import sys
import os
import runpy

# Remove executable name from args
if len(sys.argv) > 0:
    sys.argv.pop(0)

# Find tool paths
toolspath = os.path.dirname(os.path.realpath(__file__)).replace("\\", "/")
sys.path.insert(0, os.path.join(toolspath, "pyserial"))
sys.path.insert(0, os.path.join(toolspath, "esptool"))

try:
    # Run esptool as a module, supports new CLI layout
    runpy.run_module("esptool", run_name="__main__")
except ModuleNotFoundError:
    sys.stderr.write(
        "Error: esptool not found next to this upload.py tool.\n"
        "Make sure the folder 'esptool' exists under bin/.\n"
    )
    sys.exit(1)
except SystemExit as e:
    # Propagate normal esptool exits (0=ok, nonzero=error)
    sys.exit(e.code if hasattr(e, 'code') else 1)
except Exception as e:
    sys.stderr.write(f"Upload failed: {e}\n")
    sys.exit(1)
