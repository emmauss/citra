#pragma once

#include <algorithm>
#include <array>
#include <iterator>

#include "video_core/pica.h"
#include "video_core/debug_utils/debug_utils.h"
#include "video_core/shader/shader.h"

namespace Pica {

class VertexLoader {
public:
    void Setup(const Pica::Regs& regs);
    void LoadVertex(u32 base_address, int index, int vertex, Shader::InputVertex& input, DebugUtils::MemoryAccessTracker& memory_accesses);

    int GetNumTotalAttributes() const { return num_total_attributes; }

private:
    std::array<u32, 16> vertex_attribute_sources;
    std::array<u32, 16> vertex_attribute_strides{};
    std::array<Regs::VertexAttributeFormat, 16> vertex_attribute_formats;
    std::array<u32, 16> vertex_attribute_elements{};
    std::array<bool, 16> vertex_attribute_is_default;
    int num_total_attributes;
};

}  // namespace Pica
