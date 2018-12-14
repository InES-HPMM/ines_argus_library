#!/bin/bash

# compile examples
g++ -c -o  sync_buffer_cam_example.o sync_buffer_cam_example.c -Wall -I../ines_lib -std=gnu++11
g++ -o sync_buffer_cam_example sync_buffer_cam_example.o -lines_argus -L/usr/lib


g++ -c -o  sync_recorder_example.o sync_recorder_example.c -Wall -I../ines_lib -std=gnu++11
g++ -o sync_recorder_example sync_recorder_example.o -lines_argus -L/usr/lib


g++ -c -o  buffer_cam_example.o buffer_cam_example.c -Wall -I../ines_lib -std=gnu++11
g++ -o buffer_cam_example buffer_cam_example.o -lines_argus -L/usr/lib

rm *.o
