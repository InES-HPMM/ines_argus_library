# InES Argus library

## Description

This library is a C wrapper for the **Nvidia Argus library**. It makes it possible to quickly put a camera into operation and thus speeds up the development process. Within a short time video can be recorded in the h264 Format or frame buffers can be read out for further processing. It is also possible to operate several cameras synchronously. Due to abstraction, only a few function calls are required for an executable program.

## Installation
1. Install the Tegra Multimedia API with jetpack.
1. Clone this repository and checkout the release branch.
1. Step into the repository and change to the ines_lib directory `cd ines_lib`.
1. To install the ines_argus wrapper, run the create.sh script `./create.sh`. This will, create the wrapper library and copy it to `/usr/lib/`. The coresponding header file is in the current folder.
1. Now the library can be included by any c application.

## Usage

The documentation of the library and some examples of how to use it can be found in the wiki.
