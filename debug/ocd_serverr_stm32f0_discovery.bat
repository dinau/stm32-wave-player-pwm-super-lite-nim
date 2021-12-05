@echo off

@rem ---------------------------------------------------------

@set ocd_dir=D:/lpcxpresso-data/ocd-server/v10


@set BOARD_CFG=board/st_nucleo_f0.cfg

@rem ---------------------------------------------------------
@set cmd=-s %ocd_dir%/tcl -f %BOARD_CFG%
@set ocd=%ocd_dir%/openocd

@rem ---------------------------------------------------------
@:start_openocd_server
%ocd% %cmd%

