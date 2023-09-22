# $ cmake --version
cmake_minimum_required(VERSION 2.8...3.21)

# Setting the project name
project(tspd)


set(WARNING_FLAGS "-Wall -Wextra -Wpedantic -Wuninitialized -Wshadow")
set(SANITIZERS_FLAGS "-fno-omit-frame-pointer -fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined")
set(OPT_FLAGS "-O3 -march=native -flto=auto")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${WARNING_FLAGS}")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${SANITIZERS_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${OPT_FLAGS}")


# Create a sources variable with a link to all cpp files to compile
set(SOURCE_FILES
        src/main.c
        src/fstsp_vds.c
        src/leg_formulation.c
        src/utils.c
        src/tsp.c
        src/genetic.c
        src/min_max_opt.c)

# Add an executable with the above sources
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

message(STATUS "You're using ${CMAKE_SYSTEM_NAME}")
message(STATUS "You're using ${CMAKE_VERSION}")

if (WIN32)

    # put here some stuff related to WIN32

elseif (UNIX AND NOT APPLE)

    # To link CPLEX libraries and directories stored inside Blade, you must uncomment the following line
    # the latest version installed in the cluster is 12.10 (but obviously you can choose another one)

    # To link CPLEX libraries and directories stored inside your personal Linux distro just set the right path
    set(CPLEX_HOME /opt/ibm/ILOG/CPLEX_Studio221/cplex)
    # set(CPLEX_HOME /nfsd/opt/CPLEX12.10/cplex)
    
    # To link and include useful CPLEX libraries and directories, play with the following lines
    #target_link_libraries(${PROJECT_NAME} -lcplex -lm -lpthread -ldl)
    #target_link_libraries(${PROJECT_NAME} /usr/local/lib64/libnlopt.so)
    #target_link_libraries(${PROJECT_NAME} -L${CPLEX_HOME}/lib/x86-64_linux/static_pic)
    #target_include_directories(${PROJECT_NAME} PRIVATE ${CPLEX_HOME}/include/ilcplex)
    
    #target_include_directories(${PROJECT_NAME} PRIVATE ${CPLEX_HOME}/include/ilcplex ${CONCORDELIB} )
    

    target_link_libraries(${PROJECT_NAME} -lm -lpthread -ldl)
    target_link_libraries(${PROJECT_NAME} /usr/local/lib64/libnlopt.so)
    target_link_libraries(${PROJECT_NAME} ${CPLEX_HOME}/lib/x86-64_linux/static_pic/libcplex.a)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CPLEX_HOME}/include/ilcplex)
    #target_include_directories(${PROJECT_NAME} PRIVATE ${CPLEX_HOME}/include/ilcplex ${CONCORDELIB} )
    


else (APPLE)

    # To link CPLEX libraries and directories stored inside your personal MacOS distro just set the right path
    set(CPLEX_HOME /Applications/CPLEX_Studio201/cplex)
    # set(CPLEX_HOME /nfsd/opt/CPLEX12.10/cplex)
    
    # To link and include useful CPLEX libraries and directories, play with the following lines
    target_link_libraries(${PROJECT_NAME} -lcplex -lm -lpthread -ldl)
    target_link_libraries(${PROJECT_NAME} -L${CPLEX_HOME}/lib/x86-64_osx/static_pic)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CPLEX_HOME}/include/ilcplex)


endif ()
