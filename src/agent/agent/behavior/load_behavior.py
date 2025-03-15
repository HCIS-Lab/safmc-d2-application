from typing import Optional

from api import ApiRegistry, MagnetApi, MediatorApi, Px4Api
from common.logger import Logger

from .behavior import Behavior


class LoadBehavior(Behavior):

    _px4_api: Px4Api
    _magnet_api: MagnetApi
    _mediator_api: MediatorApi

    _walk_goal_radius: float

    def __init__(self):
        self._px4_api = ApiRegistry.get(Px4Api)
        self._magnet_api = ApiRegistry.get(MagnetApi)
        self._mediator_api = ApiRegistry.get(MediatorApi)

    def execute(self):
        self._magnet_api.activate_magnet()

        # TODO[lnfu] 改成不要偵測 (等過幾秒自動判定為已經拿取)
        if self._magnet_api.is_loaded:
            Logger.info("successfully loaded")

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            return "idle"

        if self._magnet_api.is_loaded:
            return "takeoff"
        return None
