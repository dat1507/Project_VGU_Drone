"""Capture only the XWayland Gazebo window, avoiding private desktop content."""
import os
import re
import subprocess
import sys


def capture(path):
    code = '''import sys
from PyQt5.QtWidgets import QApplication
app=QApplication([])
image=app.primaryScreen().grabWindow(int(sys.argv[1],16))
if image.isNull() or not image.save(sys.argv[2]):raise RuntimeError('Window capture failed')
'''
    listing=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,timeout=3,check=True).stdout
    candidates=re.findall(r'(0x[0-9a-f]+) "(?:Gazebo|Gazebo Sim)[^"\\n]*"',listing)
    if not candidates:raise RuntimeError('No Gazebo X11 window found')
    subprocess.run([sys.executable,'-c',code,candidates[0],str(path)],env=dict(os.environ,QT_QPA_PLATFORM='xcb'),check=True,timeout=5)
