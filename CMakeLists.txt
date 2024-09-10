cmake_minimum_required(VERSION 3.29)

#Name of the project
project(NEORV32_PROJECT)

#Set the C++ standard to 20
set(CMAKE_CXX_STANDARD 20)

#Path to the RISC-V32 toolchain
set(CMAKE_CXX_COMPILER /opt/riscv/bin/riscv32-unknown-elf-g++)
set(CMAKE_C_COMPILER /opt/riscv/bin/riscv32-unknown-elf-gcc)

file(GLOB_RECURSE SOURCES "../../lib/source/*.c")  # Add all .cpp files in src/ to SOURCES
file(GLOB_RECURSE HEADERS "../../lib/include/*.h")  # Add all .h files in include/ to HEADERS

#Set the flags for the RISC-V32 toolchain
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=rv32imc -mabi=ilp32")
set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -march=rv32imc -mabi=ilp32")

#Add the include directories
include_directories(../../lib/include)

#Add the common library as a custom target (adding a makefile command to cmake)
add_custom_target(
        run_make
        COMMAND make -f makefile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Building the common library"
)

#Add dependency of the executable on the common library
add_dependencies(run_make NEORV32_PROJECT)

#Add the executable
add_executable(NEORV32_PROJECT main.cpp
                camera.cpp
                camera.h
                BNO055.cpp
                BNO055.h
                screen.cpp
                screen.h
                ${HEADERS} # Add all .h files in include/ to SOURCES
             )

