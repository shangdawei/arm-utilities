# Introduction #

This wiki documents a set of utilities, libraries and header files useful for embedded and low-level ARM processor use.

# Goal #

The goal of this project is to make boot-level and embedded ARM code development, delivery and use as easy and effective as with other systems.


# Details #

This project is focused on command-line and scripted tool chain use, rather than integration with a specific GUI environment or IDE.

The code is written for Linux, and that is expected to be the primary host platform.  Modifications for other host platforms will be happily reviewed, but will likely be declined if they add significant complexity.  Lightly tested Windows and Mac support is in the source code, but the Makefile support is known to be incomplete.

A specific example of the design philosophy is the stlink-download utility.  The STLink is a USB device that presents itself as USB mass-storage (it poses as a flash drive).  The programmer/debugger interface is through vendor-specific SCSI commands.  There are multiple libraries to abstract USB and SCSI use e.g. SCSI-Generic and SCSI-Pass-Through.  None are ubiquitous, leading to extra complexity during build and installation.  Instead of using a library, we directly use the scsi-generic device.  While this resulted in higher initial development effort, the result is a more robust and more easily deployed program.

# History #

The base code was written by Donald Becker and William Carlson in December 2010 and January 2011.  MS-Windows support was added July 2011 by Anton Eltchaninov, and special contributions were later made by Hugo Becker.

We had designed an EV motor controller and chose the Atmel AVR / Arduino as the initial microcontroller platform.  After just a few minutes exploring the fun but limiting Arduino programming environment, we decided to base development on that hardware but using a traditional C-Makefile-download process.

Later we decided to switch to the more powerful ARM processor, and selected the inexpensive STM32VLDiscovery board as our core module.  We were dismayed at the very limited Windows-only support and the concomitant requirement to use a heavyweight GUI for every development activity.

These utilities, libraries and header files were the result of our desire for an equivalent quality ARM development environment.  Especially a simple development, build and delivery toolchain.  We not only wanted to develop the code, but archive it and have others use, modify, compile and download it with the same ease as with the AVR.  You'll see its influence in the register naming, call details and command line option names.



# License #

The programs and libraries may be used under the terms of the Gnu General Public License (GPL) v2 or v3.  Distribution under other terms requires an explicit license from the authors.

Certain header files and interface definitions have optional alternate licenses available.  These are explicitly noted in each file.