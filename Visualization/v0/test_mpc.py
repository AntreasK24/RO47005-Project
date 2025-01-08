import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpcDroneSolver import DroneMPCSolver  

#Plot the obstackes as spheres
def plot_sphere(ax, center, radius, color='g', alpha=0.5):
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
    ax.plot_surface(x, y, z, color=color, alpha=alpha)

def test_mpc_drone_with_visualization():

    #Initialize the drone solver
    drone_mpc_solver = DroneMPCSolver()

    #Set the initial position, target position and avoid positions
    init_pos = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])  
    target_pos = np.array([8.5, 8.5, 8.5, 0.0, 0.0, 0.0]) 
    avoid_pos0 = np.array([2.0, 2.0, 2.0])
    avoid_pos1 = np.array([3.0, 4.0, 4.0])  
    avoid_pos2 = np.array([6.0, 6.0, 5.0]) 
    avoid_pos3 = np.array([5.0, 5.0, 5.0]) 
    avoid_pos4 = np.array([7.0, 7.0, 7.0])
    avoid_pos5 = np.array([7.5, 7.5, 7.5])
    avoid_pos = np.array([avoid_pos0,avoid_pos1, avoid_pos2,avoid_pos3,avoid_pos4])

    d_min = 1.0  

    #Setup the solver
    drone_mpc_solver.setup_solver(init_pos, target_pos, avoid_pos, d_min)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')


    #Plot the drone and onstacles
    ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='r', marker='x', s=100, label='Target')
    plot_sphere(ax, avoid_pos1, d_min, color='g', alpha=0.3)
    plot_sphere(ax, avoid_pos2, d_min, color='b', alpha=0.3)
    plot_sphere(ax, avoid_pos3, d_min, color='k', alpha=0.3)
    plot_sphere(ax, avoid_pos4, d_min, color='y', alpha=0.3)
    plot_sphere(ax, avoid_pos0, d_min, color='m', alpha=0.3)
    #plot_sphere(ax, avoid_pos5, d_min, color='c', alpha=0.3)

    trajectory = [init_pos[0:3].copy()]
    velocities = [init_pos[3:6].copy()]
    accelerations = []


    #Run the simulation 
    n_steps = 200000  
    for step in range(n_steps):

        #Get acceleration from the solver and intergrate to get the new position
        control_input = drone_mpc_solver.solve(init_pos)
        init_pos[3:6] += control_input[0:3] * drone_mpc_solver.prediction_period  
        init_pos[0:3] += init_pos[3:6] * drone_mpc_solver.prediction_period  

        trajectory.append(init_pos[0:3].copy())
        velocities.append(init_pos[3:6].copy())
        accelerations.append(control_input[0:3].copy())

        ax.plot([trajectory[-2][0], trajectory[-1][0]],
                [trajectory[-2][1], trajectory[-1][1]],
                [trajectory[-2][2], trajectory[-1][2]], c='b', marker='o')

        plt.draw()
        plt.pause(0.1)

        #Print the position and time

        print("time: ", step * drone_mpc_solver.prediction_period)
        print("position", init_pos[0:3])


        #Check if the target is reached
        if np.linalg.norm(init_pos[0:3] - target_pos[0:3]) < 0.5:  
            print("Target reached!")
            break

    ax.set_xlabel('X Position [m]')
    ax.set_ylabel('Y Position [m]')
    ax.set_zlabel('Z Position [m]')
    ax.set_title('Drone Trajectory with Obstacles')
    ax.legend()

    time = np.arange(len(velocities)) * drone_mpc_solver.prediction_period
    time = time[:len(accelerations)]


    #Plot the velocity and acceleration
    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(time, [v[0] for v in velocities[:-1]], label='Vx (m/s)', c='r')
    ax1.plot(time, [v[1] for v in velocities[:-1]], label='Vy (m/s)', c='g')
    ax1.plot(time, [v[2] for v in velocities[:-1]], label='Vz (m/s)', c='b')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Velocity [m/s]')
    ax1.set_title('Drone Velocities over Time')
    ax1.legend()

    fig, ax2 = plt.subplots(figsize=(10, 6))
    ax2.plot(time, [a[0] for a in accelerations], label='Ax (m/s²)', c='r', linestyle='--')
    ax2.plot(time, [a[1] for a in accelerations], label='Ay (m/s²)', c='g', linestyle='--')
    ax2.plot(time, [a[2] for a in accelerations], label='Az (m/s²)', c='b', linestyle='--')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Acceleration [m/s²]')
    ax2.set_title('Drone Accelerations over Time')
    ax2.legend()

    plt.show()

if __name__ == "__main__":
    test_mpc_drone_with_visualization()
