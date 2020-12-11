# Any commands which fail will cause the shell script to exit immediately
set -e

# Validate Travis CI environment
if [ "$TRAVIS_BUILD_DIR" = "" ]; then
  echo "Please define 'TRAVIS_BUILD_DIR' environment variable.";
  exit 1;
fi

#export rapidassist_DIR=$TRAVIS_BUILD_DIR/third_parties/RapidAssist/install
#echo rapidassist_DIR=$rapidassist_DIR
export WIN32ARDUINO_INSTALL_DIR=$CMAKE_PREFIX_PATH/win32arduino

echo ============================================================================
echo Generating...
echo ============================================================================
cd $TRAVIS_BUILD_DIR/tests/win32arduino
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$WIN32ARDUINO_INSTALL_DIR -DWIN32ARDUINO_BUILD_TEST=ON -DWIN32ARDUINO_BUILD_SAMPLES=ON ..

echo ============================================================================
echo Compiling...
echo ============================================================================
cmake --build .
echo

echo ============================================================================
echo Installing into $INSTALL_LOCATION
echo ============================================================================
make install
echo

pwd
ls -l

# Delete all temporary environment variable created
unset GTEST_ROOT
unset rapidassist_DIR
unset INSTALL_LOCATION
