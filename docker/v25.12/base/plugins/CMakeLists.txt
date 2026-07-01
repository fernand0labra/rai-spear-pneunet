cmake_minimum_required(VERSION 3.22)

find_package(SofaFramework)
find_package(Sofa.Config REQUIRED)

sofa_add_subdirectory(plugin SofaPython3 SofaPython3)

sofa_add_subdirectory(plugin SPLIB SPLIB)
sofa_add_subdirectory(plugin STLIB STLIB)
sofa_add_subdirectory(plugin Cosserat Cosserat)
sofa_add_subdirectory(plugin SoftRobots SoftRobots)
sofa_add_subdirectory(plugin BeamAdapter BeamAdapter)
sofa_add_subdirectory(plugin ModelOrderReduction ModelOrderReduction)

sofa_add_subdirectory(plugin SofaGLFW/SofaGLFW SofaGLFW)
sofa_add_subdirectory(plugin SofaGLFW/SofaImGui SofaImGui)
sofa_add_subdirectory(application SofaGLFW/exe runSofaGLFW OFF)
