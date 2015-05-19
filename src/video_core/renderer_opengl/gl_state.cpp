// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "video_core/renderer_opengl/gl_state.h"

OpenGLState OpenGLState::cur_state;

OpenGLState::OpenGLState() {
    // These all match default OpenGL values
    cull.enabled = false;
    cull.mode = GL_BACK;

    depth.test_enabled = false;
    depth.test_func = GL_LESS;
    depth.write_mask = GL_TRUE;

    stencil.test_enabled = false;
    stencil.test_func = GL_ALWAYS;
    stencil.test_ref = 0;
    stencil.test_mask = -1;
    stencil.write_mask = -1;

    blend.enabled = false;
    blend.src_rgb_func = GL_ONE;
    blend.dst_rgb_func = GL_ZERO;
    blend.src_a_func = GL_ONE;
    blend.dst_a_func = GL_ZERO;
    blend.color.red = 0.0f;
    blend.color.green = 0.0f;
    blend.color.blue = 0.0f;
    blend.color.alpha = 0.0f;

    for (auto& texture_unit : texture_units) {
        texture_unit.enabled_2d = false;
        texture_unit.texture_2d = 0;
    }

    draw.framebuffer = 0;
    draw.vertex_array = 0;
    draw.vertex_buffer = 0;
    draw.shader_program = 0;
}

const void OpenGLState::Apply() {
    // Culling
    if (cull.enabled) {
        if (cull.enabled != cur_state.cull.enabled) {
            glEnable(GL_CULL_FACE);
        }

        if (cull.mode != cur_state.cull.mode) {
            glCullFace(cull.mode);
        }
    } else if (cull.enabled != cur_state.cull.enabled) {
        glDisable(GL_CULL_FACE);
    }

    // Depth test
    if (depth.test_enabled) {
        if (depth.test_enabled != cur_state.depth.test_enabled) {
            glEnable(GL_DEPTH_TEST);
        }

        if (depth.test_func != cur_state.depth.test_func) {
            glDepthFunc(depth.test_func);
        }
    } else if (depth.test_enabled != cur_state.depth.test_enabled) {
        glDisable(GL_DEPTH_TEST);
    }

    // Depth mask
    if (depth.write_mask != cur_state.depth.write_mask) {
        glDepthMask(depth.write_mask);
    }

    // Stencil test
    if (stencil.test_enabled) {
        if (stencil.test_enabled != cur_state.stencil.test_enabled) {
            glEnable(GL_STENCIL_TEST);
        }

        if (stencil.test_func != cur_state.stencil.test_func ||
            stencil.test_ref != cur_state.stencil.test_ref ||
            stencil.test_mask != cur_state.stencil.test_mask) {
            glStencilFunc(stencil.test_func, stencil.test_ref, stencil.test_mask);
        }
    } else if (stencil.test_enabled != cur_state.stencil.test_enabled) {
        glDisable(GL_STENCIL_TEST);
    }

    // Stencil mask
    if (stencil.write_mask != cur_state.stencil.write_mask) {
        glStencilMask(stencil.write_mask);
    }

    // Blending
    if (blend.enabled) {
        if (blend.enabled != cur_state.blend.enabled) {
            glEnable(GL_BLEND);
        }

        if (blend.color.red != cur_state.blend.color.red ||
            blend.color.green != cur_state.blend.color.green ||
            blend.color.blue != cur_state.blend.color.blue ||
            blend.color.alpha != cur_state.blend.color.alpha) {
            glBlendColor(blend.color.red, blend.color.green, blend.color.blue, blend.color.alpha);
        }

        if (blend.src_rgb_func != cur_state.blend.src_rgb_func ||
            blend.dst_rgb_func != cur_state.blend.dst_rgb_func ||
            blend.src_a_func != cur_state.blend.src_a_func ||
            blend.dst_a_func != cur_state.blend.dst_a_func) {
            glBlendFuncSeparate(blend.src_rgb_func, blend.dst_rgb_func, blend.src_a_func, blend.dst_a_func);
        }
    } else if (blend.enabled != cur_state.blend.enabled) {
        glDisable(GL_BLEND);
    }

    // Textures
    for (int i = 0; i < 3; ++i) {
        if (texture_units[i].enabled_2d) {
            if (texture_units[i].enabled_2d != texture_units[i].enabled_2d) {
                glActiveTexture(GL_TEXTURE0 + i);
                glEnable(GL_TEXTURE_2D);
            }

            if (texture_units[i].texture_2d != cur_state.texture_units[i].texture_2d) {
                glActiveTexture(GL_TEXTURE0 + i);
                glBindTexture(GL_TEXTURE_2D, texture_units[i].texture_2d);
            }
        } else if (texture_units[i].enabled_2d != cur_state.texture_units[i].enabled_2d) {
            glActiveTexture(GL_TEXTURE0 + i);
            glDisable(GL_TEXTURE_2D);
        }
    }

    // Framebuffer
    if (draw.framebuffer != cur_state.draw.framebuffer) {
        glBindFramebuffer(GL_FRAMEBUFFER, draw.framebuffer);
    }

    // Vertex array
    if (draw.vertex_array != cur_state.draw.vertex_array) {
        glBindVertexArray(draw.vertex_array);
    }

    // Vertex buffer
    if (draw.vertex_buffer != cur_state.draw.vertex_buffer) {
        glBindBuffer(GL_ARRAY_BUFFER, draw.vertex_buffer);
    }

    // Shader program
    if (draw.shader_program != cur_state.draw.shader_program) {
        glUseProgram(draw.shader_program);
    }

    cur_state = *this;
}
