import pickle
import matplotlib.pyplot as plt
import numpy as np

real_dict = pickle.load(open('results/abb/real/6640.pickle','rb'))
sim_dict = pickle.load(open('results/abb/sim/6640.pickle','rb'))


###surface plots of accleration limits, x as q2, y as q3
x_sim=[]
y_sim=[]
q1_acc_sim=[]
q2_acc_p_sim=[]
q3_acc_p_sim=[]
q2_acc_n_sim=[]
q3_acc_n_sim=[]
for key, value in sim_dict.items():
   x_sim.append(key[0])
   y_sim.append(key[1])
   q1_acc_sim.append(value[0])
   q2_acc_n_sim.append(value[2])
   q2_acc_p_sim.append(value[3])
   q3_acc_n_sim.append(value[4])
   q3_acc_p_sim.append(value[5])
###surface plots of accleration limits, x as q2, y as q3
x_real=[]
y_real=[]
q1_acc_real=[]
q2_acc_p_real=[]
q3_acc_p_real=[]
q2_acc_n_real=[]
q3_acc_n_real=[]
for key, value in real_dict.items():
   x_real.append(key[0])
   y_real.append(key[1])
   q1_acc_real.append(value[0])
   q2_acc_n_real.append(value[2])
   q2_acc_p_real.append(value[3])
   q3_acc_n_real.append(value[4])
   q3_acc_p_real.append(value[5])   
   

#####################################################################surface plots##########################################################
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf1 = ax.plot_trisurf(x_sim, y_sim, q1_acc_sim, linewidth=0, antialiased=False,label='sim')
surf2 = ax.plot_trisurf(x_real, y_real, q1_acc_real, linewidth=0, antialiased=False,label='real')
surf1._edgecolors2d = surf1._edgecolor3d
surf1._facecolors2d = surf1._facecolor3d
surf2._edgecolors2d = surf2._edgecolor3d
surf2._facecolors2d = surf2._facecolor3d

ax.set_xlabel('q2 (rad)')
ax.set_ylabel('q3 (rad)')
ax.set_zlabel('q1 acc (rad/s^2)')

plt.title('Joint1 Acceleration Limit')
plt.legend()
plt.show()

plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf1 = ax.plot_trisurf(x_sim, y_sim, q2_acc_n_sim, linewidth=0, antialiased=False,label='sim')
surf2 = ax.plot_trisurf(x_real, y_real, q2_acc_n_real, linewidth=0, antialiased=False,label='real')
surf1._edgecolors2d = surf1._edgecolor3d
surf1._facecolors2d = surf1._facecolor3d
surf2._edgecolors2d = surf2._edgecolor3d
surf2._facecolors2d = surf2._facecolor3d

ax.set_xlabel('q2 (rad)')
ax.set_ylabel('q3 (rad)')
ax.set_zlabel('q2 acc (rad/s^2)')
plt.title('Joint2 Acceleration Limit-')
plt.legend()
plt.show()


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf1 = ax.plot_trisurf(x_sim, y_sim, q2_acc_p_sim, linewidth=0, antialiased=False,label='sim')
surf2 = ax.plot_trisurf(x_real, y_real, q2_acc_p_real, linewidth=0, antialiased=False,label='real')
surf1._edgecolors2d = surf1._edgecolor3d
surf1._facecolors2d = surf1._facecolor3d
surf2._edgecolors2d = surf2._edgecolor3d
surf2._facecolors2d = surf2._facecolor3d
ax.set_xlabel('q2 (rad)')
ax.set_ylabel('q3 (rad)')
ax.set_zlabel('q2 acc (rad/s^2)')
plt.title('Joint2 Acceleration Limit+')
plt.legend()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf1 = ax.plot_trisurf(x_sim, y_sim, q3_acc_n_sim, linewidth=0, antialiased=False,label='sim')
surf2 = ax.plot_trisurf(x_real, y_real, q3_acc_p_real, linewidth=0, antialiased=False,label='real')
surf1._edgecolors2d = surf1._edgecolor3d
surf1._facecolors2d = surf1._facecolor3d
surf2._edgecolors2d = surf2._edgecolor3d
surf2._facecolors2d = surf2._facecolor3d

ax.set_xlabel('q2 (rad)')
ax.set_ylabel('q3 (rad)')
ax.set_zlabel('q3 acc (rad/s^2)')
plt.title('Joint3 Acceleration Limit-')
plt.legend()
plt.show()


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf1 = ax.plot_trisurf(x_sim, y_sim, q3_acc_p_sim, linewidth=0, antialiased=False,label='sim')
surf2 = ax.plot_trisurf(x_real, y_real, q3_acc_n_real, linewidth=0, antialiased=False,label='real')
surf1._edgecolors2d = surf1._edgecolor3d
surf1._facecolors2d = surf1._facecolor3d
surf2._edgecolors2d = surf2._edgecolor3d
surf2._facecolors2d = surf2._facecolor3d

ax.set_xlabel('q2 (rad)')
ax.set_ylabel('q3 (rad)')
ax.set_zlabel('q3 acc (rad/s^2)')
plt.title('Joint3 Acceleration Limit+')
plt.legend()
plt.show()

