from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.const import colors
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary


class Exp1(Task):

    def init_task(self) -> None:
        self.block = Shape('ball')
        success_detector = ProximitySensor('success')
        self.target = Shape('Cup')
        self.boundary = SpawnBoundary([Shape('boundary')])

        success_condition = DetectedCondition(self.block, success_detector)
        self.register_success_conditions([success_condition])

    def init_episode(self, index: int) -> List[str]:
        # setup index
        self._variation_index = index
        block_color_name, block_rgb = colors[index]
        # setup color
        self.block.set_color(block_rgb)
        self.boundary.clear()
        # sample position
        self.boundary.sample(self.target,min_rotation=(0,0,0), max_rotation=(0,0,0))
        self.boundary.sample(self.block, min_rotation=(0,0,-1.57), max_rotation=(0,0,-1.57))
        return [
            'put the %s block to the cup' % block_color_name,
            'put the %s ball to the white cup' % block_color_name,
        ]


    def variation_count(self) -> int:
        return len(colors)

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def cleanup(self) -> None:
        # Called during at the end of each episode. Remove this if not using.
        pass
