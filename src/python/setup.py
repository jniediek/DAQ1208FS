from distutils.core import setup, Extension

usb1208fs = Extension('usb1208fs', sources = ['usb1208fsmodule.c'], libraries= ['hid'], extra_objects = ['pmd.o', 'libmcchid.a'])

setup(name = "usb1208fs", version = "0.1", ext_modules = [usb1208fs])
