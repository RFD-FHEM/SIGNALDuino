# Any commands which fail will cause the shell script to exit immediately
set -e

# Validate Travis CI environment
if [ "$TRAVIS_BUILD_DIR" = "" ]; then
  echo "Please define 'TRAVIS_BUILD_DIR' environment variable.";
  exit 1;
fi
export GOOGLETEST_INSTALL_DIR=$CMAKE_PREFIX_PATH/googletest
echo ============================================================================
echo Cloning googletest into $TRAVIS_BUILD_DIR/tests/googletest
echo ============================================================================
rm -r -f $TRAVIS_BUILD_DIR/tests/googletest
mkdir -p $TRAVIS_BUILD_DIR/tests
cd $TRAVIS_BUILD_DIR/tests
git clone "https://github.com/google/googletest.git"
cd googletest
echo

echo Checking out version 1.8.1...
git checkout release-1.8.1
echo

echo ============================================================================
echo Compiling...
echo ============================================================================
mkdir -p build
cd build
cmake -DBUILD_GMOCK=OFF -DCMAKE_INSTALL_PREFIX=$GOOGLETEST_INSTALL_DIR -DCMAKE_BUILD_TYPE=Release  -Dgtest_force_shared_crt=ON  ..

cmake --build .
echo

echo ============================================================================
echo Installing into $GTEST_ROOT
echo ============================================================================
make install
echo
