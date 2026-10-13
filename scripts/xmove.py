#!/usr/bin/env python3
"""X11 のウィンドウを名前で探して移動・リサイズする(録画用に画面配置を固定するため)。

使い方: python3 xmove.py "ウィンドウ名" X Y W H
wmctrl / xdotool が無い環境でも動くよう、libX11 を ctypes で直接呼ぶ。
"""
import ctypes
import subprocess
import sys

name, x, y, w, h = sys.argv[1], *map(int, sys.argv[2:6])
tree = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True).stdout
ids = [int(line.split()[0], 16) for line in tree.splitlines()
       if f'"{name}"' in line and 'x' in line.split()[0]]
if not ids:
    sys.exit(f'ウィンドウが見つかりません: {name}')
x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
x11.XOpenDisplay.restype = ctypes.c_void_p
dpy = ctypes.c_void_p(x11.XOpenDisplay(None))
for wid in ids:
    x11.XMoveResizeWindow(dpy, ctypes.c_ulong(wid), x, y, ctypes.c_uint(w), ctypes.c_uint(h))
    x11.XRaiseWindow(dpy, ctypes.c_ulong(wid))
x11.XFlush(dpy)
x11.XCloseDisplay(dpy)
print(f'{name}: {len(ids)} 個のウィンドウを ({x}, {y}) {w}x{h} へ移動')
