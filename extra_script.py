try:
    import configparser
except ImportError:
    import ConfigParser as configparser

Import("env")

# access to global build environment
#print(env)

# view all build environment
#print(env.Dump())

# to build date str and buildname
import datetime
date = datetime.datetime.now()
date = date.strftime("%y") + date.strftime("%m") + date.strftime("%d")

# for config.get
# config = configparser.ConfigParser()
# config.read("platformio.ini")
# build_version = config.get("env", "BUILD_FLAGS")


# name from env:project | example: [env:nano_bootl_old_CC1101] 
build_name = env['PIOENV']

# Build versions with a point are difficult to process
# System uses dot for file extension
build_release = "350_dev"
build_version = "v" + build_release + date

# write project hex, bin, elf to nano_bootl_old_CC1101_v350_dev_20200811.hex
env.Replace(PROGNAME="%s" % build_name + "_" + "%s" % build_version)