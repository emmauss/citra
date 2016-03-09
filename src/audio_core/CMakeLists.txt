set(SRCS
            audio_core.cpp
            codec.cpp
            hle/effects.cpp
            hle/dsp.cpp
            hle/final.cpp
            hle/pipe.cpp
            hle/source.cpp
            null_sink.cpp
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
            sink.h
            null_sink.h
            )

create_directory_groups(${SRCS} ${HEADERS})

add_library(audio_core STATIC ${SRCS} ${HEADERS})