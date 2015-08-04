A set of utilities and libraries for low-level and embedded ARM processor application development.

The most commonly used utility is stlink-download, a stand-alone command line DUDE (download/upload/debug) utility for the STM STLink USB-JTAG interface, as found on the STM32VLDiscovery and related development boards.

The support files include header and library files for a more efficient alternative to the commercial Windows support packages.  The header files encourage a direct-register-write programming style that largely eliminates the library call overhead for setting up devices, reducing code size and increasing speed.

The run-time and linker support is similarly efficiency and clarity oriented.  The only "frill" is enabling all device register clocks by default to eliminate the most common programming error with the STM32 series.

Explore our wiki page [ARMUtilitiesOverview](ARMUtilitiesOverview.md) to get more information about the project.
