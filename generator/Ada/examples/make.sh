#!/bin/bash

mkdir build

gnatmake -a --create-missing-dirs attitude.adb -D build -I..
gnatmake -a --create-missing-dirs param_request_list.adb -D build -I..
gnatmake -a --create-missing-dirs param_set_sr0_params -D build -I..
