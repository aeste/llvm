include_directories( ${CMAKE_CURRENT_BINARY_DIR}/.. 
                     ${CMAKE_CURRENT_SOURCE_DIR}/.. )

add_llvm_library(LLVMT3RASInfo
  T3RASTargetInfo.cpp
  )

add_dependencies(LLVMT3RASInfo T3RASCommonTableGen)
