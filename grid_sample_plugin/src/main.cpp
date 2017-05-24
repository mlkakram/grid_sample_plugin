#include "grid_sample_node.h"

extern "C" Plugin* createPlugin() {
    return new GridSamplerNode::GridSamplerNode();
}

extern "C" std::string getType() {
    return "grid_sampler_plugin";
}
