## grid_sample_plugin

Example Usage:

```bash

import grid_sample_client
result = grid_sample_client.GridSampleClient.computePreGrasps(10)
grasps = grid_sample_client.GridSampleClient.evaluatePreGrasps(result.grasps)

```