import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='MyEnv-v0',
    entry_point='pleg.envs.environment:RobotEnv',
)
