#!/bin/bash

meson --buildtype debug build-debug
meson --buildtype debugoptimized build-debugoptimized
meson --buildtype release build-release
