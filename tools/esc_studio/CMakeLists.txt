cmake_minimum_required(VERSION 3.22)
project(JescStudio VERSION 0.1 LANGUAGES C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Quick QuickControls2 SerialPort)
qt_standard_project_setup(REQUIRES 6.5)

add_library(studio_core STATIC
    src/Dictionary.cpp src/Dictionary.h src/ProtocolSession.cpp src/ProtocolSession.h
    src/ParameterScheme.cpp src/ParameterScheme.h
    src/ScopeData.cpp src/ScopeData.h src/CaptureEngine.cpp src/CaptureEngine.h
    src/ScopeViewModel.cpp src/ScopeViewModel.h src/DemoScope.cpp src/DemoScope.h
    src/ScopeRecording.cpp src/ScopeRecording.h
    src/ScopeAnalysis.cpp src/ScopeAnalysis.h
    ../../canopen/od_wire.c ../../canopen/od_wire.h)
target_include_directories(studio_core PUBLIC src ../../canopen)
target_link_libraries(studio_core PUBLIC Qt6::Core)

add_library(studio_plot STATIC src/WaveformItem.cpp src/WaveformItem.h)
target_link_libraries(studio_plot PUBLIC studio_core Qt6::Quick)
qt_add_executable(esc_studio src/main.cpp src/Backend.cpp src/Backend.h)
target_link_libraries(esc_studio PRIVATE studio_plot Qt6::Gui Qt6::Quick Qt6::QuickControls2 Qt6::SerialPort)
foreach(qml Main ParameterPage MotorPage ScopePage)
    set_source_files_properties(qml/${qml}.qml PROPERTIES QT_RESOURCE_ALIAS ${qml}.qml)
endforeach()
qt_add_qml_module(esc_studio URI JescStudio VERSION 1.0
    QML_FILES qml/Main.qml qml/ParameterPage.qml qml/MotorPage.qml qml/ScopePage.qml)
qt_add_resources(esc_studio dictionary PREFIX "/" BASE ../..
    FILES ../../canopen/object_dictionary.csv)

include(CTest)
if(BUILD_TESTING)
    find_package(Qt6 REQUIRED COMPONENTS Test QuickTest)
    qt_add_executable(scope_probe tests/scope_probe.cpp)
    set_target_properties(scope_probe PROPERTIES WIN32_EXECUTABLE FALSE)
    target_link_libraries(scope_probe PRIVATE studio_core Qt6::SerialPort)
    qt_add_executable(studio_tests tests/test_studio.cpp)
    set_target_properties(studio_tests PROPERTIES WIN32_EXECUTABLE FALSE)
    target_link_libraries(studio_tests PRIVATE studio_core Qt6::Test)
    target_compile_definitions(studio_tests PRIVATE
        JESC_DICTIONARY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../canopen/object_dictionary.csv")
    add_test(NAME studio_core COMMAND studio_tests -o qt-tests.txt,txt)
    qt_add_executable(scope_tests tests/test_scope.cpp)
    target_link_libraries(scope_tests PRIVATE studio_core Qt6::Test)
    add_test(NAME scope_core COMMAND scope_tests -o scope-tests.txt,txt)
    if(TARGET Qt6::GuiPrivate)
        qt_add_executable(scope_render_tests tests/test_render.cpp)
        target_link_libraries(scope_render_tests PRIVATE studio_plot Qt6::Test Qt6::GuiPrivate)
        add_test(NAME scope_rhi COMMAND scope_render_tests -o render-tests.txt,txt)
        set_tests_properties(scope_rhi PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen;QT_QUICK_BACKEND=;QML_DISABLE_DISK_CACHE=1")
    endif()
    qt_add_executable(studio_ui_tests tests/test_ui.cpp src/Backend.cpp src/Backend.h)
    target_link_libraries(studio_ui_tests PRIVATE studio_plot Qt6::QuickTest Qt6::QuickControls2 Qt6::SerialPort)
    target_compile_definitions(studio_ui_tests PRIVATE QUICK_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/qml")
    qt_add_resources(studio_ui_tests test_dictionary PREFIX "/" BASE ../..
        FILES ../../canopen/object_dictionary.csv)
    add_test(NAME studio_ui COMMAND studio_ui_tests -o ui-tests.txt,txt)
    set_tests_properties(studio_ui PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen;QT_QUICK_BACKEND=software;QML_DISABLE_DISK_CACHE=1")
endif()
if(MSVC)
    target_compile_options(studio_core PRIVATE /W4 /utf-8)
    target_compile_options(studio_plot PRIVATE /W4 /utf-8)
    target_compile_options(esc_studio PRIVATE /W4 /utf-8)
else()
    target_compile_options(studio_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(studio_plot PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(esc_studio PRIVATE -Wall -Wextra -Wpedantic)
endif()
