cd build
cmake .. 
cmake --build .
./trajectory_gen | python3 ../grapher.py
cd ..