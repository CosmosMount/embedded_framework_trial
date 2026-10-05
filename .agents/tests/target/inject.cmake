# Opt-in target test build; leaves the production application unchanged.
function(pnx_target_test)
    get_target_property(sources pnx_embedded SOURCES)
    list(FILTER sources EXCLUDE REGEX "/app/app\\.cpp$")
    list(APPEND sources "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/core.cpp")
    set_property(TARGET pnx_embedded PROPERTY SOURCES "${sources}")
endfunction()
cmake_language(DEFER CALL pnx_target_test)
