# CMake generated Testfile for 
# Source directory: C:/Users/jeremy.quintin/Controller/C_Controller
# Build directory: C:/Users/jeremy.quintin/Controller/C_Controller
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test(unit-tests "C:/Users/jeremy.quintin/Controller/C_Controller/Debug/test_C_Controller.exe.exe")
  set_tests_properties(unit-tests PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/jeremy.quintin/Controller/C_Controller/CMakeLists.txt;100;add_test;C:/Users/jeremy.quintin/Controller/C_Controller/CMakeLists.txt;0;")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test(unit-tests "C:/Users/jeremy.quintin/Controller/C_Controller/Release/test_C_Controller.exe.exe")
  set_tests_properties(unit-tests PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/jeremy.quintin/Controller/C_Controller/CMakeLists.txt;100;add_test;C:/Users/jeremy.quintin/Controller/C_Controller/CMakeLists.txt;0;")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test(unit-tests "C:/Users/jeremy.quintin/Controller/C_Controller/MinSizeRel/test_C_Controller.exe.exe")
  set_tests_properties(unit-tests PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/jeremy.quintin/Controller/C_Controller/CMakeLists.txt;100;add_test;C:/Users/jeremy.quintin/Controller/C_Controller/CMakeLists.txt;0;")
elseif("${CTEST_CONFIGURATION_TYPE}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test(unit-tests "C:/Users/jeremy.quintin/Controller/C_Controller/RelWithDebInfo/test_C_Controller.exe.exe")
  set_tests_properties(unit-tests PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/jeremy.quintin/Controller/C_Controller/CMakeLists.txt;100;add_test;C:/Users/jeremy.quintin/Controller/C_Controller/CMakeLists.txt;0;")
else()
  add_test(unit-tests NOT_AVAILABLE)
endif()
subdirs("lowlevel")
