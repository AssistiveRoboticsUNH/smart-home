macro(set_cache_variable VAR_NAME VAR_DESCRIPTION)
    set(${VAR_NAME} ${${VAR_NAME}} CACHE INTERNAL ${VAR_DESCRIPTION})
    message(STATUS "Set ${VAR_NAME} to ${${VAR_NAME}}.")
endmacro()


# Code coverage

if(BUILD_CPR_TESTS AND GENERATE_COVERAGE)
    set(CMAKE_BUILD_TYPE COVERAGE CACHE INTERNAL "Coverage enabled build")
    message(STATUS "Enabling gcov support")
    if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        set(COVERAGE_FLAG "--coverage")
    endif()
    set(CMAKE_CXX_FLAGS_COVERAGE
        "-g -O0 ${COVERAGE_FLAG} -fprofile-arcs -ftest-coverage"
        CACHE STRING "Flags used by the C++ compiler during coverage builds."
        FORCE)
    set(CMAKE_C_FLAGS_COVERAGE
        "-g -O0 ${COVERAGE_FLAG} -fprofile-arcs -ftest-coverage"
        CACHE STRING "Flags used by the C compiler during coverage builds."
        FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
        ""
        CACHE STRING "Flags used for linking binaries during coverage builds."
        FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE
        ""
        CACHE STRING "Flags used by the shared libraries linker during coverage builds."
        FORCE)
    mark_as_advanced(
        CMAKE_CXX_FLAGS_COVERAGE
        CMAKE_C_FLAGS_COVERAGE
        CMAKE_EXE_LINKER_FLAGS_COVERAGE
        CMAKE_SHARED_LINKER_FLAGS_COVERAGE)
endif()


# Curl configuration

if(USE_SYSTEM_CURL)
    find_package(CURL)
    set(CURL_FOUND TRUE)
endif()
if(NOT USE_SYSTEM_CURL OR NOT CURL_FOUND)
    message(STATUS "Not using system Curl, using built-in curl project instead.")
    option(BUILD_TESTING "Set to ON to build cURL tests." OFF)
    option(BUILD_CURL_EXE "Set to ON to build cURL executable." OFF)
    add_subdirectory(curl)
    set(CURL_FOUND TRUE)
    set(CURL_LIBRARIES libcurl)
    set(CURL_INCLUDE_DIRS
        ${CURL_SOURCE_DIR}/include
        ${CURL_BINARY_DIR}/include/curl)
        
    # Group under the "external" project folder in IDEs such as Visual Studio.
    if(BUILD_CURL_EXE)
        set_property(TARGET curl PROPERTY FOLDER "external")
    endif()
        
    set_property(TARGET libcurl PROPERTY FOLDER "external")
endif()

set_cache_variable(CURL_FOUND "Set if libcurl is found or built")
set_cache_variable(CURL_LIBRARIES "Location of libcurl")
set_cache_variable(CURL_INCLUDE_DIRS "Location of curl include files")


# Mongoose configuration

if(BUILD_CPR_TESTS)
    message(STATUS "Building mongoose project for test support.")
    add_subdirectory(mongoose)
    set(MONGOOSE_FOUND TRUE)
    set(MONGOOSE_LIBRARIES mongoose)
    set(MONGOOSE_INCLUDE_DIRS ${mongoose_SOURCE_DIR})

    set_cache_variable(MONGOOSE_FOUND "Set if libmongoose was found or built")
    set_cache_variable(MONGOOSE_LIBRARIES "Location of libmongoose")
    set_cache_variable(MONGOOSE_INCLUDE_DIRS "Location of mongoose include files")
    
    # Group under the "external" project folder in IDEs such as Visual Studio.
    set_property(TARGET mongoose PROPERTY FOLDER "external")
endif()
