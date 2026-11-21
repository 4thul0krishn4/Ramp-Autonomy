#!/usr/bin/env python3
"""Tiny X11 helper (ctypes, no extra packages): find the top-level Gazebo GUI window and resize/move it, or print its id.
Usage: x11_window.py id                  -> print the window id of the "gz sim gui" client window
       x11_window.py resize W H [X Y]    -> ask the window manager (EWMH _NET_MOVERESIZE_WINDOW) to resize/move it"""
import ctypes, ctypes.util, sys, time
x = ctypes.cdll.LoadLibrary(ctypes.util.find_library('X11'))
x.XOpenDisplay.restype = ctypes.c_void_p; x.XOpenDisplay.argtypes = [ctypes.c_char_p]
x.XDefaultRootWindow.restype = ctypes.c_ulong; x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
x.XInternAtom.restype = ctypes.c_ulong; x.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
x.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)), ctypes.POINTER(ctypes.c_uint)]
x.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]
x.XGetClassHint.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p]
x.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.c_void_p]
d = x.XOpenDisplay(None)
if not d: sys.exit('cannot open display')
root = x.XDefaultRootWindow(d)


class XClassHint(ctypes.Structure):
    _fields_ = [('res_name', ctypes.c_char_p), ('res_class', ctypes.c_char_p)]


def walk(w, out, depth=0):
    r, p, ch, n = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.POINTER(ctypes.c_ulong)(), ctypes.c_uint()
    if not x.XQueryTree(d, w, ctypes.byref(r), ctypes.byref(p), ctypes.byref(ch), ctypes.byref(n)): return
    for i in range(n.value):
        c = ch[i]; nm = ctypes.c_char_p()
        if x.XFetchName(d, c, ctypes.byref(nm)) and nm.value:
            hint = XClassHint()
            cls = (hint.res_name, hint.res_class) if x.XGetClassHint(d, c, ctypes.byref(hint)) else (None, None)
            out.append((c, nm.value.decode(errors='ignore'), cls))
        walk(c, out, depth + 1)


def find():
    out = []; walk(root, out)
    # the real client window: name "Gazebo Sim" with class gz-sim-gui
    for c, nm, cls in out:
        if nm == 'Gazebo Sim' and cls[0] == b'gz-sim-gui': return c
    return None


class XClientMessage(ctypes.Structure):
    _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong), ('send_event', ctypes.c_int), ('display', ctypes.c_void_p), ('window', ctypes.c_ulong),
                ('message_type', ctypes.c_ulong), ('format', ctypes.c_int), ('data', ctypes.c_long * 5)]


cmd = sys.argv[1]
w = find()
if w is None: sys.exit('Gazebo GUI window not found')
if cmd == 'id':
    print(hex(w))
elif cmd == 'resize':
    W, H = int(sys.argv[2]), int(sys.argv[3]); X, Y = (int(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else (0, 0)
    ev = XClientMessage(); ev.type = 33; ev.send_event = 1; ev.display = d; ev.window = w
    ev.message_type = x.XInternAtom(d, b'_NET_MOVERESIZE_WINDOW', 0); ev.format = 32
    # flags: gravity(0)=default, x/y/w/h present (bits 8-11), source indication = pager(2) bits 12-15
    ev.data[0] = (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11) | (2 << 12); ev.data[1], ev.data[2], ev.data[3], ev.data[4] = X, Y, W, H
    x.XSendEvent(d, root, 0, (1 << 20) | (1 << 19), ctypes.byref(ev)); x.XFlush.argtypes = [ctypes.c_void_p]; x.XFlush(d)
    time.sleep(1.0); print('requested', W, H)
