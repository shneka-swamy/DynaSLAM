echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
mkdir -p conan
pushd conan
conan install ../.. --build=missing --settings build_type=Release
popd
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_MODULE_PATH=$PWD/conan -DCMAKE_PREFIX_PATH=$PWD/conan -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building DynaSLAM ..."

mkdir -p build
cd build
mkdir -p conan
pushd conan
conan install ../.. --build=missing --settings build_type=Debug
conan install ../.. --build=missing --settings build_type=Release
popd
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
cd ..
