@echo off



@rem ---------------------------------------------------------
@set elf_name=nucleo_f030r8.elf
@set ocd_dir=D:/lpcxpresso-data/ocd-server/v10
@set INSIGHT_DIR=c:/arm-none-eabi-insight/bin
@set GDB_INIT=init.gdb
@set BOARD_CFG=board/st_nucleo_f0.cfg

@rem ---------------------------------------------------------
@set cmd=-s %ocd_dir%/tcl -f %BOARD_CFG%
@set ocd=%ocd_dir%/openocd
@
@rem ---------------------------------------------------------
@:write
@rem %ocd% %cmd% -c "mt_flash %elf_name%"
@
@rem ---------------------------------------------------------
@:start_openocd_server
@start %ocd% %cmd%

rem ---------------------------------------------------------
:start_insight
%INSIGHT_DIR%/arm-none-eabi-insight.exe  -x %GDB_INIT% --se=%elf_name%
