Closest point on Mesh

Author: Patrick Korczak

This package is locally linked to third party libraries including the Eigen library for C++ vector algebra computations and tinyply 2.3 for ply file processing. CMake must be used to compile the source code.

Eigen is free open source software under the MPL2-license. tinyply 2.3 is in the public domain. 

tinyply 2.3
https://github.com/ddiakopoulos/tinyply

Eigen
https://eigen.tuxfamily.org

Instructions:

1. Create a build folder in the same directory where CMakelists.txt and this README file is found. On linux run: "mkdir build".

2. To compile the source code enter the build folder. On linux run: "cd build". Then run the necessary commands to build the source code and generate an executable file. On linux run: "cmake ..". After the build files have been successfully generated, run: "make". Note that CMake is required to build and compile the code.

3. You can start the program by running the executable file. In linux: "./project_sources".

4. The program allows you to input a custom query point or the default (1.5,1.5,1.5). Then you must specify the maximum search distance.

NOTE: The ply file to be read by the program is assigned in main.cpp. It's currently set to "icosahredron.ply" in the assets folder. To load a different ply file, change the file name in the const string filename variable in main.cpp. Remember that the pathway to the ply file is relative to the build folder.
