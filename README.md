## grid_sample_plugin

Example Usage:
In one terminal:
```bash
roslaunch grid_sample_plugin grid_sample_plugin.launch
```

Then, in another terminal:

```bash
import graspit_commander
gc = graspit_commander.GraspitCommander()
gc.loadWorld('plannerMug')

import grid_sample_client
pre_grasps = grid_sample_client.GridSampleClient.computePreGrasps(10)
grasps = grid_sample_client.GridSampleClient.evaluatePreGrasps(pre_grasps.grasps)

```
