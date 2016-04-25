file(GLOB SIMPLEAI_SRC src/*.cpp)
file(GLOB SIMPLEAI_HDR include/*.h)

add_library(continuum SHARED ${CONTINUUM_SRC} ${CONTINUUM_HDR})
target_include_directories(continuum PRIVATE
  ./include
  ../external
  ../steerlib/include
  ../util/include
)
target_link_libraries(continuum steerlib util)
add_dependencies(continuum steerlib util)

install(TARGETS continuum
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(FILES ${SIMPLEAI_HDR} DESTINATION include/continuum)
