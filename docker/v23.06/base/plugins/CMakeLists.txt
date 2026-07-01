cmake_minimum_required(VERSION 3.22)

find_package(SofaFramework)

sofa_add_subdirectory(plugin SofaPython3 SofaPython3)

sofa_add_subdirectory(plugin SPLIB SPLIB)
sofa_add_subdirectory(plugin STLIB STLIB)
sofa_add_subdirectory(plugin Cosserat Cosserat)
sofa_add_subdirectory(plugin SoftRobots SoftRobots)
sofa_add_subdirectory(plugin BeamAdapter BeamAdapter)
sofa_add_subdirectory(plugin ModelOrderReduction ModelOrderReduction)
