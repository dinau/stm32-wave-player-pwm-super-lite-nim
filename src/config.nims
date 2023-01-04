import std/[os, strutils, strscans]

if NimVersion > "1.6.0":
    --hint: "Conf:off" # nim-1.6.0 or later
    --hint:"Name:off" # nim-1.6.8 or later

--verbosity: 1

const
    TARGET = "start"
    SRC_DIR = "."

    # Define binutils
    OBJCOPY = "arm-none-eabi-objcopy"
    OBJDUMP{.used.} = "arm-none-eabi-objdump"
    OBJSIZE = "arm-none-eabi-size"

    # build path
    BUILD_DIR = "BINHEX"
    OUT_NAME = getCurrentDir().splitFile.name
    target_build_path = os.joinPath(BUILD_DIR, OUT_NAME)
    target_src_path = os.joinPath(SRC_DIR, TARGET)

switch "o", joinPath(BUILD_DIR, OUT_NAME) & ".elf"

# for gcc
switch "arm.standalone.gcc.exe", "arm-none-eabi-gcc"
switch "arm.standalone.gcc.linkerexe", "arm-none-eabi-gcc"
switch "arm.any.gcc.exe", "arm-none-eabi-gcc"
switch "arm.any.gcc.linkerexe", "arm-none-eabi-gcc"
#
switch "gcc.options.always", ""
switch "gcc.options.debug", ""
switch "gcc.options.size", "-Os"
switch "gcc.options.speed", "-Os"

# for clang
switch "arm.standalone.clang.exe", "clang"
switch "arm.standalone.clang.linkerexe", "arm-none-eabi-gcc"
# for  Linux
switch "gcc.options.linker", "-static"
#
# Memory manager and signal handler
when true: # Both true and false are ok.
    switch "mm", "arc"
    switch "os", "any"
    switch "d", "noSignalHandler"
    switch "d", "useMalloc"
    --passL: "-specs=nosys.specs"
else:
    switch "mm", "none"
    switch "os", "standalone"

# Size optimize
--opt: size
switch "d", "danger"
switch "panics", "on"
#
#
--threads:off # for nim-2.0 or later
--cpu: arm

# Silence warnings of 'not GC-safe'. ???!
#switch "threadAnalysis:off
#--listcmd # Verbose display gcc commnand line.
switch "passC", "-mthumb -ffunction-sections -fdata-sections -Os -g"
--passL: "-nostartfiles"
--passL: "-Wl,--gc-sections"
--passL: "--specs=nano.specs"
--passC: "-Wno-discarded-qualifiers"

#--passL:"-lc -lm -lgcc"
# https://stackoverflow.com/questions/72218980/gcc-v12-1-warning-about-serial-compilation
--passC: "-flto"
--passL: "-flto"
switch "passL", "-Wl,-Map=$#" % [target_build_path & ".map"]

switch "nimcache", ".nimcache"

#start: xpfintf integer version
switch "path", "lib/xprintf"
switch "passC", "-I../../../../src/lib/xprintf"

# path
switch "path", "."
switch "path", "lib"

# Define tasks
task clean, "Clean target":
    echo "Removed $#, $#" % [BUILD_DIR, nimcacheDir()]
    rmDir BUILD_DIR
    rmDir nimcacheDir()

task make, "Build target":
    # Compile target
    exec "nim c " & target_src_path
    # Show target size
    exec "$# $#.elf" % [OBJSIZE, target_build_path]
    # Generate *.hex file
    exec "$# -O ihex $#.elf $#.hex" % [OBJCOPY, target_build_path, target_build_path]
    # Generate *.bin file
    exec "$# -O binary -S $#.elf $#.bin" % [OBJCOPY, target_build_path, target_build_path]
    # Gen. assembler list files
    # Can't redirect to file using exec proc.
    #exec "$# -hS $#.elf > $#.lst" % [OBJDUMP,target_build_path,target_build_path]
    #exec "$# -hd $#.elf > $#.lst2" % [OBJDUMP,target_build_path,target_build_path]

#task w, "Upload to Flash":
#    makeTask()
#    exec "$#/bin/$# -c arduino -C $# -P $# -p m328p -b $#  -u -e -U flash:w:$#.elf:a" %
#        [AVR_GCC_DIR,"avrdude".toExe,CONF,COM_PORT,AVRDUDE_BAUDRATE,target_build_path]

task hex, "Copy hex to hex dir":
    mkDir("hex")
    cpFile(target_build_path & ".hex", os.joinPath("hex", OUT_NAME & ".hex"))


const WRITER_EXE = "STM32_Programmer_CLI".toExe()

task w, "Upload to Flash":
    makeTask()
    const devID = staticRead(os.absolutePath("devid.txt")).strip
    let
        (strOut, _) = gorgeEx(WRITER_EXE & " -c port=SWD")
        seqOut = strOut.split("\n")
    var chipDevID: string
    var cmd = ""
    for line in seqOut:
        if line.scanf("Device ID$s:$s$+", chipDevID) and
            find(chipDevID.toLower(), devID) >= 0:
            cmd = WRITER_EXE & " -c port=SWD -d " & target_build_path & ".hex" & " -Rst"
            exec cmd
    if cmd == "":
        echo "===================="
        echo "Error ! Device check"
        echo "===================="

