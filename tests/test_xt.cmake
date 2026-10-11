# The public transmit files tests/corpus/fetch.py downloads are built when present (skipped otherwise).
target_compile_definitions(${target} PRIVATE OPAD_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
set_tests_properties(${name} PROPERTIES TIMEOUT 600)
