#!/bin/bash
make clean
make libines_argus.so -j4
sudo cp libines_argus.so /usr/lib/
make clean
