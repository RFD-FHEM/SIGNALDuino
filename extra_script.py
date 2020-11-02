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



### copy file to subfolder
## this relief works from the 2nd run
movehelp = "no"

if movehelp == "yes":
 from pathlib import Path
 import shutil
 import platform

 current_path = env['PROJECT_BUILD_DIR']
 
 ## OS check ##
 if platform.system() == "Windows":
  current_file = env['PROJECT_BUILD_DIR'] + "\\" +  build_name + "\\" + env['PIOENV'] + "_" + "%s" % build_version
  new_file_bin = env['PROJECT_BUILD_DIR'] + "\\" + env['PIOENV'] + "_" + "%s" % build_version + ".bin"
  new_file_hex = env['PROJECT_BUILD_DIR'] + "\\" + env['PIOENV'] + "_" + "%s" % build_version + ".hex"

 if platform.system() == "Linux":
  current_file = env['PROJECT_BUILD_DIR'] + "/" +  build_name + "/" + env['PIOENV'] + "_" + "%s" % build_version
  new_file_bin = env['PROJECT_BUILD_DIR'] + "/" + env['PIOENV'] + "_" + "%s" % build_version + ".bin"
  new_file_hex = env['PROJECT_BUILD_DIR'] + "/" + env['PIOENV'] + "_" + "%s" % build_version + ".hex"
  
 current_file_bin = Path(current_file + ".bin")
 current_file_hex = Path(current_file + ".hex")

 ## if bin file exists, copy to subfolder
 if current_file_bin.is_file():
  print("- bin file exists -")
  print(current_file_bin)
  print("- bin file to -")
  print(new_file_bin)
  shutil.copyfile(str(current_file_bin), str(new_file_bin))

 ## if hex file exists, copy to subfolder
 if current_file_hex.is_file():
  print("- hex file exists -")
  print(current_file_hex)
  print("- hex file to -")
  print(new_file_hex)
  shutil.copyfile(str(current_file_hex), str(new_file_hex))
