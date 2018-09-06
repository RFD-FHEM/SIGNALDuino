# Any commands which fail will cause the shell script to exit immediately
set -e

# Validate Travis CI environment
if [ "$TRAVIS_BUILD_DIR" = "" ]; then
  echo "Please define 'TRAVIS_BUILD_DIR' environment variable.";
  exit 1;
fi

export TEST_PROJECT_DIR=$TRAVIS_BUILD_DIR/tests



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

ls -l
pwd

# Delete all temporary environment variable created
unset GTEST_ROOT
unset rapidassist_DIR
unset win32arduino_DIR
unset LIBRARY_TEMP_DIR
