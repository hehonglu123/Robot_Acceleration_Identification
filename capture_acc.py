from abb_motion_program_exec import *
from pandas import *
import json, pickle, copy
from scipy.interpolate import interp1d

from robot_def import *
from tes_env import *
import matplotlib.pyplot as plt

#########################################################robot specific control functions##################################################
def movej_abb(q,v,z,args):
	###abb moveabsj function
	#q: joint angle in radians
	#v: tcp speed
	#z: blending zone
	#args: (motionprogram,)
	(mp,)=args
	mp.MoveAbsJ(jointtarget(np.degrees(q),[0]*6),speeddata(v,9999999,9999999,999999),zonedata(False,z,1.5*z,1.5*z,0.15*z,1.5*z,0.15*z))
	return mp

def execute_abb(args):
	###execute function
	#args:(client,motionprogram)
	(client,mp)=args
	log_results = client.execute_motion_program(mp)

	return log_results.data[:,0],log_results.data[:,2:8]



######################################################acceleration capture functions######################################################
def linear_interp(x,y):
	###avoid divided by 0 problem
	x,unique_indices=np.unique(x,return_index=True)
	if (len(unique_indices)<len(y)-2):
		print('Duplicate in interpolate, check timestamp')
	y=y[unique_indices]
	f=interp1d(x,y.T)
	x_new=np.linspace(x[0],x[-1],len(x))
	return x_new, f(x_new).T
def moving_average(a, n=11, padding=False):
	#n needs to be odd for padding
	if padding:
		a=np.hstack(([np.mean(a[:int(n/2)])]*int(n/2),a,[np.mean(a[-int(n/2):])]*int(n/2)))
	ret = np.cumsum(a, axis=0)
	ret[n:] = ret[n:] - ret[:-n]
	return ret[n - 1:] / n
def lfilter(x, y):
	x,y=linear_interp(x,y)
	n=10
	y1=moving_average(y,n)
	y2=moving_average(np.flip(y,axis=0),n)

	return x[int(n/2):-int(n/2)+1], (y1+np.flip(y2,axis=0))/2


def get_acc(timestamp,curve_exe_js,q,joint):
	###filter
	timestamp, curve_exe_js=lfilter(timestamp, curve_exe_js)
	###get qdot, qddot
	qdot_all=np.gradient(curve_exe_js,axis=0)/np.tile([np.gradient(timestamp)],(6,1)).T
	qddot_all=np.gradient(qdot_all,axis=0)/np.tile([np.gradient(timestamp)],(6,1)).T

	qddot_sorted=np.sort(qddot_all[:,joint])

	qddot_max_p=np.average(qddot_sorted[-5:])
	qddot_max_n=-np.average(qddot_sorted[:5])

	return qddot_max_p, qddot_max_n

def exec(q_d,joint,displacement,MotionProgramFunc,robot_client):
	###move joint at q_d configuration
	q_init=copy.deepcopy(q_d)
	q_end=copy.deepcopy(q_d)
	q_init[joint]+=displacement
	q_end[joint]-=displacement
	
	mp = MotionProgramFunc()
	movej_abb(q_d,200,0,(mp,))

	j_init=jointtarget(np.degrees(q_init),[0]*6)
	j_end=jointtarget(np.degrees(q_end),[0]*6)
	for i in range(4):
		movej_abb(q_init,999999,10,(mp,))
		movej_abb(q_end,999999,10,(mp,))
	
	timestamp,curve_exe_js=execute_abb((robot_client,mp))

	return get_acc(timestamp,curve_exe_js,q_d,joint)



def capture_acc():
	robot=robot_obj('ABB_6640_180_255','config/abb_6640_180_255_robot_default_config.yml')
	MotionProgramFunc=MotionProgram
	robot_client=MotionProgramExecClient(base_url="http://127.0.0.1:80")


	resolution=0.05 ###rad
	displacement=0.02

	dict_table={}
	directions=[-1,1]

	#####################first & second joint acc both depends on second and third joint#####################################
	for q2 in np.arange(robot.lower_limit[1]+displacement+0.01,robot.upper_limit[1]-displacement-0.01,resolution):
		for q3 in np.arange(robot.lower_limit[2]+displacement+0.01,robot.upper_limit[2]-displacement-0.01,resolution):
			###initialize keys, and desired pose

			dict_table[(q2,q3)]=[0]*6 		###[+j1,-j1,+j2,-j2,+j3,-j3]
			q_d=[0,q2,q3,0,0,0]

			#measure first joint first
			qddot_max,_=exec(q_d,0,displacement,MotionProgramFunc,robot_client)
			###update dict
			dict_table[(q2,q3)][0]=qddot_max
			dict_table[(q2,q3)][1]=qddot_max

			for joint in range(1,3):
				###move first q2 and q3
				qddot_max_p,qddot_max_n=exec(q_d,joint,displacement,MotionProgramFunc,robot_client)
				###update dict
				dict_table[(q2,q3)][2*joint]=qddot_max_p
				dict_table[(q2,q3)][2*joint+1]=qddot_max_n


	with open(r'test.txt','w+') as f:
		f.write(str(dict_table))
	pickle.dump(dict_table, open('test.pickle','wb'))

def capture_acc_collision():
	robot_name='ABB_6640_180_255'
	robot=robot_obj(robot_name,'config/abb_6640_180_255_robot_default_config.yml')
	MotionProgramFunc=MotionProgram
	robot_client=MotionProgramExecClient(base_url="http://192.168.55.1:80")
	t=Tess_Env('config/urdf/abb_cell')				#create obj

	resolution=0.05 ###rad
	displacement=0.02

	dict_table={}
	directions=[-1,1]

	#####################first & second joint acc both depends on second and third joint#####################################
	for q2 in np.arange(robot.lower_limit[1]+displacement+0.01,robot.upper_limit[1]-displacement-0.01,resolution):
		for q3 in np.arange(robot.lower_limit[2]+displacement+0.01,robot.upper_limit[2]-displacement-0.01,resolution):
			###check for collision
			if t.check_collision_single(robot_name,np.array([0,q2,q3,0,0,0])):
				continue
			###initialize keys, and desired pose
			dict_table[(q2,q3)]=[0]*6 		###[+j1,-j1,+j2,-j2,+j3,-j3]
			q_d=[0,q2,q3,0,0,0]

			#measure first joint first
			qddot_max,_=exec(q_d,0,displacement,MotionProgramFunc,robot_client)
			###update dict
			dict_table[(q2,q3)][0]=qddot_max
			dict_table[(q2,q3)][1]=qddot_max

			for joint in range(1,3):
				###move first q2 and q3
				qddot_max_p,qddot_max_n=exec(q_d,joint,displacement,MotionProgramFunc,robot_client)
				###update dict
				dict_table[(q2,q3)][2*joint]=qddot_max_p
				dict_table[(q2,q3)][2*joint+1]=qddot_max_n


	with open(r'test.txt','w+') as f:
		f.write(str(dict_table))
	pickle.dump(dict_table, open('test.pickle','wb'))


if __name__ == '__main__':
	capture_acc_collision()