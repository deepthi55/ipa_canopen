ADD_LIBRARY(canopen STATIC
  cia_402_constants.h
  canopen_profiles.h
  canopen.cpp
  cia_402.cpp
  )

install (TARGETS canopen 
#  LIBRARY DESTINATION lib)
  ARCHIVE DESTINATION lib)

install (FILES 
  canopen.h
  cia_402.h
  schunkErrors.h
  cia_402_constants.h
  DESTINATION include)

