# Description

This project is a test for `k-line` automotive diagnostics.
`klogger` - logging of `k-line` activity;
`hd` - Honda CR-V III SRS ECU simple diagnostics

# How to build

This project has been created for [CodeBlocks](https://www.codeblocks.org/).
Just install and open project files.

# Hardware

You need [J2534](https://www.boschdiagnostics.com/j2534-faq) diag tool. I use [Tactrix OpenPort 2.0](https://www.tactrix.com), this is very good hardware, supporting `j2534` standard.

# How to run

This is console application, you have to run it from console tool: `cmd` or `PowerShell` (Windows).

## klogger

Run without parameters to get help:

```
klogger [logfile] {switches}

    [logfile]          log file to write
    /b [baudrate] baud rate to use
    /p {none,odd,even} parity to use (defaults to none)
    /c {k,l,aux} channel to use (defaults to K)
    /t [timeout] timeout in ms to determine end of message (defaults to 20ms)
```

run with file name parameter to sniff k-line on default values.

## hd

Current version has no parameters. If you have ECU on the line it will get identifiers from ECU, read currnet DTC, clear DTC. This is the default sequence, if you want to change it, change the code.


