@echo off
rem ARCH.BAT
rem
rem    Script for returning architecture on the executing platform.
rem    This Batch file applies to all Windows platforms, and determines
rem    the architecture in the environment variable BINARCH.
rem
rem    Example:  The following is a fragment of a Batch file that uses
rem              the arch script to obtain the system architecture: 
rem 
rem              setlocal
rem              call %MATLAB%\bin\util\arch.bat
rem              call %MATLAB%\bin\%BINARCH%\matlab.bat
rem              endlocal
rem
rem    Copyright 1984-2010 The MathWorks, Inc.
rem    $Revision: 1.1.6.5 $  $Date: 2010/05/13 17:07:44 $
rem  __________________________________________________________________________
rem

rem Export results in BINARCH environment variable.
rem Not using setlocal/endlocal.  Batch files that call this one
rem can use setlocal/endlocal as shown above to limit the scope
rem of the BINARCH variable defined here.

if NOT (%1) == () (
	set BINARCH=unknown
	call :throwError "No arguments are allowed."
)

rem Use a heuristic to determine MATLAB arch.
if defined MATLAB_ARCH (
  if "%MATLAB_ARCH%" == "win32" (
    set BINARCH=win32
  ) else if "%MATLAB_ARCH%" == "win64" (
    set BINARCH=win64
  ) else (
    call :throwError "Unsupported platform."
  )
) else if "%PROCESSOR_ARCHITECTURE%" == "AMD64" (
    call :setBinArchOn64
) else if "%PROCESSOR_ARCHITEW6432%" == "AMD64" (
    call :setBinArchOn64
) else if "%PROCESSOR_ARCHITECTURE%" == "x86" (
    set BINARCH=win32
) else (
    call :throwError "Unsupported platform."
)
goto :eof

rem Refine test for running 32-bit MATLAB on win64.
:setBinArchOn64
if exist %~dps0\..\win64\matlab.exe (
    set BINARCH=win64
) else if exist %~dps0\..\win32\matlab.exe (
    set BINARCH=win32
) else (
	call :throwError "Unsupported platform."
)
goto :eof

rem Function that prints error string input in quotes.
:throwError
	echo Error: ARCH: %~1 1>&2
	set BINARCH=unknown

