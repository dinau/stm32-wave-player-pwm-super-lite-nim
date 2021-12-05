import os,strutils

--hint:"Conf:off"
--verbosity:1

const
    TARGET    = "start"
    SRC_DIR   = "."

    # Define binutils
    OBJCOPY   = "arm-none-eabi-objcopy"
    OBJDUMP   = "arm-none-eabi-objdump"
    OBJSIZE   = "arm-none-eabi-size"

    # build path
    BUILD_DIR = "BINHEX"
    OUT_NAME = getCurrentDir().splitFile.name
    target_build_path = os.joinPath(BUILD_DIR,OUT_NAME)
    target_src_path   = os.joinPath(SRC_DIR,TARGET)

switch "o", joinPath(BUILD_DIR, OUT_NAME) & ".elf"

# for gcc
switch "arm.standalone.gcc.exe","arm-none-eabi-gcc"
switch "arm.standalone.gcc.linkerexe","arm-none-eabi-gcc"
switch "arm.any.gcc.exe","arm-none-eabi-gcc"
switch "arm.any.gcc.linkerexe","arm-none-eabi-gcc"
#
switch "gcc.options.always" , ""
switch "gcc.options.debug" ,""
switch "gcc.options.size" ,"-Os"
switch "gcc.options.speed" ,"-Os"

# for clang
switch "arm.standalone.clang.exe","clang"
switch "arm.standalone.clang.linkerexe","arm-none-eabi-gcc"
# for  Linux
switch "gcc.options.linker", "-static"
#
# Memory manager and signal handler
when false:
    switch "gc","arc"
    switch "os","any"
    switch "d","noSignalHandler"
    switch "d","useMalloc"
    --passL:"-specs=nosys.specs"
else:
    switch "gc","none"
    switch "os","standalone"

# Size optimize
--opt:size
switch "d","danger"
switch "panics", "on"
#
--cpu:arm

# Silence warnings of 'not GC-safe'. ???!
#switch "threadAnalysis:off
#--listcmd # Verbose display gcc commnand line.
--passC:"-mthumb -ffunction-sections -fdata-sections -Os -g"
--passL:"-Wl,--gc-sections"
--passC:"--specs=nano.specs"
--passC:"-Wno-discarded-qualifiers"
--passL:"-lc -lm -lgcc"
switch "passL","-Wl,-Map=$#" % [target_build_path & ".map"]

switch "nimcache",".nimcache"

#start: xpfintf integer version
switch "path","lib/xprintf"
switch "passC","-I../../../../src/lib/xprintf"

# path
switch "path","."
switch "path","lib"

# Define tasks
task clean, "Clean target":
    echo "Removed $#, $#" % [BUILD_DIR,nimcacheDir()]
    rmDir BUILD_DIR
    rmDir nimcacheDir()

task make, "Build target":
    # Compile target
    exec "nim c " & target_src_path
    # Show target size
    exec "$# $#.elf" % [OBJSIZE,target_build_path]
    # Generate *.hex file
    exec "$# -O ihex $#.elf $#.hex" % [OBJCOPY,target_build_path,target_build_path]
    # Generate *.bin file
    exec "$# -O binary -S $#.elf $#.bin" % [OBJCOPY,target_build_path,target_build_path]
    # Generate *.lst file with source code
    var (output,res) = gorgeEx( "$# -hSC $#.elf" % [OBJDUMP,target_build_path])
    writeFile(target_build_path & ".lst", output)
    # Generate *.lst2 file without source code
    (output,res) = gorgeEx( "$# -hdC -fax $#.elf" % [OBJDUMP,target_build_path])
    writeFile(target_build_path & ".lst2", output)

#task w, "Upload to Flash":
#    makeTask()
#    exec "$#/bin/$# -c arduino -C $# -P $# -p m328p -b $#  -u -e -U flash:w:$#.elf:a" %
#        [AVR_GCC_DIR,"avrdude".toExe,CONF,COM_PORT,AVRDUDE_BAUDRATE,target_build_path]

task hex, "Copy hex to hex dir":
    mkDir("hex")
    cpFile(target_build_path & ".hex", os.joinPath("hex",OUT_NAME & ".hex"))


