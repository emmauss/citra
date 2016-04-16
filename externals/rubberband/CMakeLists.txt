set(SRCS
        rubberband/src/audiocurves/CompoundAudioCurve.cpp
        rubberband/src/audiocurves/ConstantAudioCurve.cpp
        rubberband/src/audiocurves/HighFrequencyAudioCurve.cpp
        rubberband/src/audiocurves/PercussiveAudioCurve.cpp
        rubberband/src/audiocurves/SilentAudioCurve.cpp
        rubberband/src/audiocurves/SpectralDifferenceAudioCurve.cpp
        rubberband/src/base/Profiler.cpp
        rubberband/src/dsp/AudioCurveCalculator.cpp
        rubberband/src/dsp/FFT.cpp
        rubberband/src/dsp/Resampler.cpp
        rubberband/src/kissfft/kiss_fft.c
        rubberband/src/kissfft/kiss_fftr.c
        rubberband/src/RubberBandStretcher.cpp
        rubberband/src/speex/resample.c
        rubberband/src/StretchCalculator.cpp
        rubberband/src/StretcherChannelData.cpp
        rubberband/src/StretcherImpl.cpp
        rubberband/src/StretcherProcess.cpp
        rubberband/src/system/Allocators.cpp
        rubberband/src/system/sysutils.cpp
        rubberband/src/system/Thread.cpp
        rubberband/src/system/VectorOpsComplex.cpp
        )

SET(HEADERS
        rubberband/src/audiocurves/CompoundAudioCurve.h
        rubberband/src/audiocurves/ConstantAudioCurve.h
        rubberband/src/audiocurves/HighFrequencyAudioCurve.h
        rubberband/src/audiocurves/PercussiveAudioCurve.h
        rubberband/src/audiocurves/SilentAudioCurve.h
        rubberband/src/audiocurves/SpectralDifferenceAudioCurve.h
        rubberband/src/base/Profiler.h
        rubberband/src/base/RingBuffer.h
        rubberband/src/base/Scavenger.h
        rubberband/src/dsp/AudioCurveCalculator.h
        rubberband/src/dsp/FFT.h
        rubberband/src/dsp/MovingMedian.h
        rubberband/src/dsp/Resampler.h
        rubberband/src/dsp/SampleFilter.h
        rubberband/src/dsp/SincWindow.h
        rubberband/src/dsp/Window.h
        rubberband/src/float_cast/float_cast.h
        rubberband/src/kissfft/kiss_fft.h
        rubberband/src/kissfft/kiss_fftr.h
        rubberband/src/kissfft/_kiss_fft_guts.h
        rubberband/src/pommier/neon_mathfun.h
        rubberband/src/pommier/sse_mathfun.h
        rubberband/src/speex/speex_resampler.h
        rubberband/src/StretchCalculator.h
        rubberband/src/StretcherChannelData.h
        rubberband/src/StretcherImpl.h
        rubberband/src/system/Allocators.h
        rubberband/src/system/sysutils.h
        rubberband/src/system/Thread.h
        rubberband/src/system/VectorOps.h
        rubberband/src/system/VectorOpsComplex.h
        rubberband/rubberband/rubberband-c.h
        rubberband/rubberband/RubberBandStretcher.h
        )

add_library(rubberband STATIC ${SRCS} ${HEADERS})

target_include_directories(rubberband PRIVATE rubberband/src)
target_include_directories(rubberband PRIVATE rubberband)

set_property(TARGET rubberband APPEND PROPERTY COMPILE_DEFINITIONS USE_SPEEX)
if(APPLE)
    set_property(TARGET rubberband APPEND PROPERTY COMPILE_DEFINITIONS HAVE_VDSP)
    set_property(TARGET rubberband APPEND PROPERTY COMPILE_DEFINITIONS USE_PTHREADS)
    target_link_libraries(rubberband "-framework Accelerate")
elseif(MSVC)
    set_property(TARGET rubberband APPEND PROPERTY COMPILE_DEFINITIONS USE_KISSFFT)
    set_property(TARGET rubberband APPEND PROPERTY COMPILE_DEFINITIONS __MSVC__)
    set_property(TARGET rubberband APPEND PROPERTY COMPILE_DEFINITIONS WIN32)
else()
    set_property(TARGET rubberband APPEND PROPERTY COMPILE_DEFINITIONS USE_KISSFFT)
    set_property(TARGET rubberband APPEND PROPERTY COMPILE_DEFINITIONS USE_PTHREADS)
endif()
