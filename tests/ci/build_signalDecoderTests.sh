# Any commands which fail will cause the shell script to exit immediately
set -e

# Validate Travis CI environment
if [ "$TRAVIS_BUILD_DIR" = "" ]; then
  echo "Please define 'TRAVIS_BUILD_DIR' environment variable.";
  exit 1;
fi

export GTEST_ROOT=$TRAVIS_BUILD_DIR/third_parties/googletest/install
export rapidassist_DIR=$TRAVIS_BUILD_DIR/third_parties/RapidAssist/install
export win32arduino_DIR=$TRAVIS_BUILD_DIR/install
export TEST_PROJECT_DIR=$TRAVIS_BUILD_DIR/tests/testSignalDecoder/



echo ============================================================================
echo Generating...
echo ============================================================================
cd $TEST_PROJECT_DIR
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..

echo ============================================================================
echo Compiling...
echo ============================================================================
cmake --build .
echo

# Delete all temporary environment variable created
unset GTEST_ROOT
unset rapidassist_DIR
unset win32arduino_DIR
unset LIBRARY_TEMP_DIR
