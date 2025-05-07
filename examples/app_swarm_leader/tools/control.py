import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
import cflib.crtp
import numpy as np

uris = [
    'radio://0/80/1M/E7E7E7E7E1',
]

positions = {uri: (0, 0, 0) for uri in uris}

def log_pos(scf, uri):
    log_conf = LogConfig(name='Pose', period_in_ms=100)
    log_conf.add_variable('pose.drone_x', 'float')
    log_conf.add_variable('pose.drone_y', 'float')
    log_conf.add_variable('pose.drone_z', 'float')

    with SyncLogger(scf, log_conf) as logger:
        for entry in logger:
            data = entry[1]
            x, y, z = data['pose.drone_x'], data['pose.drone_y'], data['pose.drone_z']
            positions[uri] = (x, y, z)

def animate(i):
    xs, ys, zs = zip(*positions.values())
    scat.set_offsets(np.c_[xs, ys])
    ax.set_title(f"Crazyflie Swarm Positions (t={i})")

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    fig, ax = plt.subplots()
    scat = ax.scatter([], [], c='blue')
    ax.set_xlim(-2, 4)
    ax.set_ylim(-2, 4)

    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel(log_pos)
        ani = FuncAnimation(fig, animate, interval=200)
        plt.show()