# zstd for the Pia payload (de)compression in the cport brain. Portable C source (ZSTD_DISABLE_ASM),
# builds for RISC-V (C6) as well as Xtensa.
#
# 2026-10-01: sources are now VENDORED in-repo at ./lib (previously referenced by an absolute path
# into frlg-ldn-trade-esp32, which broke any clean checkout). KIZUNA_ZSTD_LIB still overrides the
# location if needed. The EXISTS guard + empty-component fallback stays so modes that never use zstd
# still configure even if ./lib is somehow absent.
if(DEFINED ENV{KIZUNA_ZSTD_LIB})
    set(ZLIB "$ENV{KIZUNA_ZSTD_LIB}")
else()
    # CMAKE_CURRENT_LIST_DIR (this file's dir), NOT CMAKE_CURRENT_SOURCE_DIR: the latter resolves to
    # the project build dir during IDF's early component-expansion phase, which registered zstd EMPTY
    # and silently dropped cport's REQUIRES zstd. CMAKE_CURRENT_LIST_DIR is stable in both phases.
    set(ZLIB "${CMAKE_CURRENT_LIST_DIR}/lib")
endif()

if(EXISTS "${ZLIB}/zstd.h")
    file(GLOB ZSTD_SRCS "${ZLIB}/common/*.c" "${ZLIB}/compress/*.c" "${ZLIB}/decompress/*.c")
    idf_component_register(SRCS ${ZSTD_SRCS} INCLUDE_DIRS "${ZLIB}")
    # ZSTD_DECODER_INTERNAL_BUFFER: the decoder's literal scratch buffer, 64 KB by default, which makes a
    # decoder context 96 KB (ZSTD_estimateDCtxSize). Pia frames are a few hundred bytes, so 1 KB is plenty
    # and the context shrinks to ~31 KB -- the difference between fitting and not fitting on the ESP32-C3
    # (kizuna-duo docs/66 section 3.3, docs/70). cport/pia_zstd.c keeps ONE such context for the whole session.
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ZSTD_DISABLE_ASM=1 XXH_NAMESPACE=ZSTD_ ZSTD_DECODER_INTERNAL_BUFFER=1024)
    target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-maybe-uninitialized -Wno-error=maybe-uninitialized)
else()
    message(STATUS "zstd: sources not found at ${ZLIB} - registering an empty component")
    idf_component_register()
endif()
