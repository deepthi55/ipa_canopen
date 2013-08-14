ADD_LIBRARY(canopen STATIC
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
  DESTINATION include)

