// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "common/common_types.h"

#include "video_core/pica.h"

#include "generated/gl_3_2_core.h"

namespace PicaToGL {

inline GLenum WrapMode(Pica::Regs::TextureConfig::WrapMode mode) {
    static const GLenum wrap_mode_table[] = {
        GL_CLAMP_TO_EDGE,  // WrapMode::ClampToEdge
        0,                 // Unknown
        GL_REPEAT,         // WrapMode::Repeat
        GL_MIRRORED_REPEAT // WrapMode::MirroredRepeat
    };

    // Range check table for input
    if (mode >= sizeof(wrap_mode_table) / sizeof(GLenum)) {
        LOG_CRITICAL(Render_OpenGL, "Unknown texture wrap mode %d", mode);
        UNREACHABLE();

        return GL_CLAMP_TO_EDGE;
    }

    GLenum gl_mode = wrap_mode_table[mode];

    // Check for dummy values indicating an unknown mode
    if (gl_mode == 0) {
        LOG_CRITICAL(Render_OpenGL, "Unknown texture wrap mode %d", mode);
        UNIMPLEMENTED();

        return GL_CLAMP_TO_EDGE;
    }

    return gl_mode;
}

inline GLenum BlendFunc(u32 factor) {
    static const GLenum blend_func_table[] = {
        GL_ZERO,                     // BlendFactor::Zero
        GL_ONE,                      // BlendFactor::One
        GL_SRC_COLOR,                // BlendFactor::SourceColor
        GL_ONE_MINUS_SRC_COLOR,      // BlendFactor::OneMinusSourceColor
        GL_DST_COLOR,                // BlendFactor::DestColor
        GL_ONE_MINUS_DST_COLOR,      // BlendFactor::OneMinusDestColor
        GL_SRC_ALPHA,                // BlendFactor::SourceAlpha
        GL_ONE_MINUS_SRC_ALPHA,      // BlendFactor::OneMinusSourceAlpha
        GL_DST_ALPHA,                // BlendFactor::DestAlpha
        GL_ONE_MINUS_DST_ALPHA,      // BlendFactor::OneMinusDestAlpha
        GL_CONSTANT_COLOR,           // BlendFactor::ConstantColor
        GL_ONE_MINUS_CONSTANT_COLOR, // BlendFactor::OneMinusConstantColor
        GL_CONSTANT_ALPHA,           // BlendFactor::ConstantAlpha
        GL_ONE_MINUS_CONSTANT_ALPHA, // BlendFactor::OneMinusConstantAlpha
        GL_SRC_ALPHA_SATURATE,       // BlendFactor::SourceAlphaSaturate
    };

    // Range check table for input
    if (factor >= sizeof(blend_func_table) / sizeof(GLenum)) {
        LOG_CRITICAL(Render_OpenGL, "Unknown blend factor %d", factor);
        UNREACHABLE();

        return GL_ONE;
    }

    return blend_func_table[factor];
}

inline GLenum CompareFunc(u32 func) {
    static const GLenum compare_func_table[] = {
        GL_NEVER,    // CompareFunc::Never
        GL_ALWAYS,   // CompareFunc::Always
        GL_EQUAL,    // CompareFunc::Equal
        GL_NOTEQUAL, // CompareFunc::NotEqual
        GL_LESS,     // CompareFunc::LessThan
        GL_LEQUAL,   // CompareFunc::LessThanOrEqual
        GL_GREATER,  // CompareFunc::GreaterThan
        GL_GEQUAL,   // CompareFunc::GreaterThanOrEqual
    };

    // Range check table for input
    if (func >= sizeof(compare_func_table) / sizeof(GLenum)) {
        LOG_CRITICAL(Render_OpenGL, "Unknown compare function %d", func);
        UNREACHABLE();

        return GL_ALWAYS;
    }

    return compare_func_table[func];
}

} // namespace
