set(SRCS
            audio_core.cpp
            codec.cpp
            hle/effects.cpp
            hle/dsp.cpp
            hle/final.cpp
            hle/pipe.cpp
            hle/source.cpp
            null_sink.cpp
            time_stretch.cpp
            )

set(HEADERS
            audio_core.h
            codec.h
            hle/common.h
            hle/effects.h
            hle/dsp.h
            hle/final.h
            hle/pipe.h
            hle/source.h
            null_sink.h
            sink.h
            time_stretch.h
            )

if(SDL2_FOUND)
    set(SRCS ${SRCS} sdl2_sink.cpp)
    set(HEADERS ${HEADERS} sdl2_sink.h)
    include_directories(${SDL2_INCLUDE_DIR})
endif()

include_directories(../../externals/soundtouch)

create_directory_groups(${SRCS} ${HEADERS})

add_library(audio_core STATIC ${SRCS} ${HEADERS})
target_link_libraries(audio_core SoundTouch)

if(SDL2_FOUND)
    target_link_libraries(audio_core ${SDL2_LIBRARY})
endif()