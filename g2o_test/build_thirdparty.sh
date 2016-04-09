echo "Configuring and building Third party library:"
echo "Start......"
echo "build ArUco first."
cd Thirdparty/aruco_lib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
echo "build ArUco done."

echo "build g2o."
cd ../../g2o_lib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
echo "build g2o done."
