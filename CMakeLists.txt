CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# configuration
set(SCRIPT	"ultra")
set(FILES	
    "ngUltra/memory.xml" 
    "ngUltra/pipeline.xml" 
    "ngUltra/cache.xml" 
    "ngUltra/bht.xml"
)

set(SOURCES
	hook.cpp
    preFetchAnalysis.cpp
	armCortexR52F_time.h
    
	ngultraR52BBTime.cpp

)
set(ARM_HOME	"${CMAKE_SOURCE_DIR}/../armv7t")
set(GLISS_HOME 	"${CMAKE_SOURCE_DIR}/../gliss2")

# looking for required resources
find_file(IRG arm.irg PATHS "${ARM_HOME}")
find_program(GLISS_ATTR "gliss-attr" PATHS "${GLISS_HOME}/gep")


# script
project(${SCRIPT})


# look for OTAWA
if(NOT OTAWA_CONFIG)
	find_program(OTAWA_CONFIG otawa-config DOC "path to otawa-config" PATHS "${GLISS_HOME}/../../../otawa-install/bin")
	if(NOT OTAWA_CONFIG)
		message(FATAL_ERROR "ERROR: otawa-config is required !")
	endif()
endif()
message(STATUS "otawa-config at ${OTAWA_CONFIG}")

if(PREFIX)
	set(OTAWA_PREFIX ${PREFIX})
else()
	execute_process(COMMAND "${OTAWA_CONFIG}" --prefix OUTPUT_VARIABLE OTAWA_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
set(CMAKE_INSTALL_PREFIX "${OTAWA_PREFIX}")
execute_process(COMMAND "${OTAWA_CONFIG}" --cflags OUTPUT_VARIABLE OTAWA_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${OTAWA_CONFIG}" -p ${SCRIPT}.eld --libs --rpath -p ${SCRIPT}.eld  OUTPUT_VARIABLE OTAWA_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${OTAWA_CONFIG}" --plugdir  OUTPUT_VARIABLE PLUGDIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${OTAWA_CONFIG}" --scriptdir  OUTPUT_VARIABLE SCRIPTDIR OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")

add_custom_command(
	OUTPUT "armCortexR52F_time.h" DEPENDS "otawa_time.tpl" "otawa_time.nmp" ${IRG}
	COMMAND ${GLISS_ATTR}
	ARGS ${IRG} -o armCortexR52F_time.h -a cortexR52F_time -t "otawa_time.tpl" -d "SET_TIME(R52F_time_unknown);" -e otawa_time.nmp -p
	VERBATIM
)



# plugin definition
include_directories("${CMAKE_SOURCE_DIR}" "${ARM_HOME}/include")
add_library(${SCRIPT} MODULE ${SOURCES})
set_property(TARGET ${SCRIPT} PROPERTY PREFIX "")
set_property(TARGET ${SCRIPT} PROPERTY COMPILE_FLAGS ${OTAWA_CFLAGS})
target_link_libraries(${SCRIPT} "${OTAWA_LDFLAGS}")


# installation
install(TARGETS ${SCRIPT} LIBRARY DESTINATION "${PLUGDIR}/otawa")
#install(FILES "${SCRIPT}.eld" DESTINATION "${PLUGDIR}/otawa")
install(FILES "${SCRIPT}.eld" DESTINATION "${PLUGDIR}/otawa")
install(FILES "${SCRIPT}.osx" DESTINATION "${SCRIPTDIR}")
foreach(FILE ${FILES})
	install(FILES ${CMAKE_SOURCE_DIR}/${FILE} DESTINATION "${SCRIPTDIR}/ngUltra")
endforeach()

# tests
if(WITH_TEST)
	add_subdirectory(test)
endif()
