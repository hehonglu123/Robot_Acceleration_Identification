import pickle
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

# dic = ''
# with open(r'test.txt','r') as f:
#          for i in f.readlines():
#             dic=i #string
# dic = eval(dic) # this is orignal dict with instace dict

dic = pickle.load(open('results/abb/sim/6640/6640.pickle','rb'))

# dic = pickle.load(open('test.pickle','rb'))


###surface plots of accleration limits, x as q2, y as q3
x=[]
y=[]
x3=[]
y3=[]
q1_acc=[]
q2_acc_p=[]
q3_acc_p=[]
q2_acc_n=[]
q3_acc_n=[]
for key, value in dic.items():
   x.append(key[0])
   y.append(key[1])
   q1_acc.append(value[0])
   q2_acc_n.append(value[2])
   q2_acc_p.append(value[3])
   if key[1]>-3:
      x3.append(key[0])
      y3.append(key[1])
      q3_acc_n.append(value[4])
      q3_acc_p.append(value[5])
   
   

#####################################################################get acc from q###########################################################
# q=np.array([2,0,-1,1,3,4])
# xy=np.array([x,y]).T
# idx=np.argmin(np.linalg.norm(xy-q[1:3],axis=1))
# print('q2,q3 at: ',x[idx],y[idx])
# print('acc: ',q1_acc[idx],q2_acc[idx],q3_acc[idx],47.29253791291949,39.49167516506145,54.32806813314554)

#####################################################################surface plots##########################################################
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_trisurf(x, y, q1_acc, linewidth=0, antialiased=False)
ax.set_xlabel('q2 (rad)')
ax.set_ylabel('q3 (rad)')
ax.set_zlabel('q1 acc (rad/s^2)')

plt.title('Joint1 Acceleration Limit', x=0.5, y=0.95)
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_trisurf(x, y, q2_acc_n, linewidth=0, antialiased=False, label='-')
surf = ax.plot_trisurf(x, y, q2_acc_p, linewidth=0, antialiased=False, label='+')

ax.set_xlabel('q2 (rad)')
ax.set_ylabel('q3 (rad)')
ax.set_zlabel('q2 acc (rad/s^2)')
plt.title('Joint2 Acceleration Limit')
fake2Dline = mpl.lines.Line2D([0],[0], linestyle="none", c='b', marker = 'o')
fake2Dline2 = mpl.lines.Line2D([0],[0], linestyle="none", c='orange', marker = 'o')
ax.legend([fake2Dline,fake2Dline2], ['- direction','+ direction'], numpoints = 1)
plt.show()





fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_trisurf(x3, y3, q3_acc_n, linewidth=0, antialiased=False)
surf = ax.plot_trisurf(x3, y3, q3_acc_p, linewidth=0, antialiased=False)

ax.set_xlabel('q2 (rad)')
ax.set_ylabel('q3 (rad)')
ax.set_zlabel('q3 acc (rad/s^2)')
plt.title('Joint3 Acceleration Limit')
fake2Dline = mpl.lines.Line2D([0],[0], linestyle="none", c='b', marker = 'o')
fake2Dline2 = mpl.lines.Line2D([0],[0], linestyle="none", c='orange', marker = 'o')
ax.legend([fake2Dline,fake2Dline2], ['- direction','+ direction'], numpoints = 1)
plt.show()