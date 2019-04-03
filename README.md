# freepascal
Free Pascal Compiler - an open source Object Pascal compiler. This is an unofficial mirror of the FPC SubVersion repository and is used for experimental testing of new AVR controller related features.

Changes:
* Patch (see bug #34721) to generate correct code sequence for writing to 16 bit I/O registers. 
* Added to compiler: avrxmega3 subarch, atmega 3208, 3209, 4808, 4809 memory layout
* Added to rtl/embedded: Updated MakeFile to recognize avrxmega3 subarch. Also added subarch to unit output dir so that multiple subarchitectures can co-exist (example fpclocal.cfg shows how to modify config file to point to correct folder structure for AVR)
* Added to rtl/embedded/avr: atmega(3208 - 4809).pp files with just the interrupt vectors and normal startup code, no register & port definitions yet
* Patch from bug #33914 to fix variable addresses saved in debug information
* Assembler routines for intrinsic integer math operations
Compile cross compiler for either by opening compiler/ppcavr.lpi in Lazarus (quick compile, compiler will be created under compiler/avr/pp) or by the following command: ```make buildbase OS_TARGET=embedded CPU_TARGET=avr SUBARCH=avrxmega3 FPC=~/fpc/3.0.4/compiler/ppcx86 BINUTILSPREFIX=avr-```Adjust BINUTILSPREFIX and FPC for your specific system.
* RTL make file modified so that targets requiring a subarch output compiled units to rtl/units/$fpctarget-$fpcsubarch.
* Fix to make RTL stack error checking properly.

This may give an error when compiling the RTL because not all the makefiles have been updated.  Change into the rtl/embedded folder, then continue to build the rtl: ```make OS_TARGET=embedded CPU_TARGET=avr SUBARCH=avrxmega3 FPC=~/fpc/avr/compiler/avr/pp BINUTILSPREFIX=avr-```

The generated units should end up in rtl/units/avr-embedded-avrxmega3.  Change subarch to build rtl for other controllers.
