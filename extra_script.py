try:
    import configparser
except ImportError:
    import ConfigParser as configparser

Import("env")

### access to global build environment
#print(env)

### view all build environment
#print(env.Dump())

### to build date str and buildname
import datetime
import subprocess
import os
import platform
date = datetime.datetime.now().strftime("%y%m%d")

# for config.get
# config = configparser.ConfigParser()
# config.read("platformio.ini")
# build_version = config.get("env", "BUILD_FLAGS")

# name from env:project | example: [env:nano_bootl_old_CC1101] 
build_name = env['PIOENV']

# Build versions with a point are difficult to process
# System uses dot for file extension

reftype = os.environ.get('GITHUB_REF_TYPE','local')
basetag = (
    subprocess.check_output(["git", "describe", "--tags", "--first-parent", "--abbrev=1"])
    .strip()
    .decode("utf-8")
)

if (reftype == 'branch') :
    build_version = basetag+os.environ.get('GITHUB_HEAD_REF')+"+"+date
elif (reftype == 'tag') :
    build_version = os.environ.get('GITHUB_REF_NAME',basetag+"+"+date)
else:
    build_version = basetag+"+"+date

# write project hex, bin, elf like SIGANALduino_esp32_CC1101_3.5.0-11-gcc56+230423.bin
env.Replace(PROGNAME="SIGNALduino_{0}_{1}".format(build_name,build_version))

env.Append(CPPDEFINES=[
    ("PROGVERS",env.StringifyMacro(build_version)),
])


#Copy functions has to be finished, it does currently not work
def copy_firmware(source, target, env):

    from pathlib import Path
    import shutil
    filename= source
    if env["PIOPLATFORM"] == "espressif32" or env["PIOPLATFORM"] == "espressif8266":
        filename = source + ".bin"
    else:
        filename = source + ".hex"
    
    print (filename)
    ## if bin file exists, copy to subfolder
    # shutil.copyfile(str(filename), str(BUILD_DIR))


#if platform.system() == "Windows":
#    env.AddPostAction("$BUILD_DIR\\${PROGNAME}", copy_firmware)

#if platform.system() == "Linux":
#    env.AddPostAction("$BUILD_DIR/${PROGNAME}", copy_firmware)
