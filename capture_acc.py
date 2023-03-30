from abb_motion_program_exec import *
from dx200_motion_program_exec_client import *

from pandas import *
import json, pickle, copy
from scipy.interpolate import interp1d

from robots_def import *
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

	return log_results.data[:,0],np.radians(log_results.data[:,2:8])

def movej_motoman(q,v,z,args):
	###abb moveabsj function
	#q: joint angle in radians
	#v: tcp speed
	#z: blending zone
	#args: (motionprogram,)
	(client,)=args
	client.MoveJ(np.degrees(q),min(v,100),z)
	return client

def execute_motoman(args):
	###execute function
	#args:(client,motionprogram)
	(client,)=args
	timestamp, curve_exe_js = client.execute_motion_program()

	return timestamp, curve_exe_js


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

def exec_motion_abb(q_d,joint,displacement,MotionProgramFunc,robot,robot_client):
	###move joint at q_d configuration
	q_init=copy.deepcopy(q_d)
	q_end=copy.deepcopy(q_d)
	q_init[joint]+=displacement
	q_end[joint]-=displacement
	
	mp = MotionProgramFunc()
	mp=movej(q_d,200,0,(mp,))

	j_init=jointtarget(np.degrees(q_init),[0]*6)
	j_end=jointtarget(np.degrees(q_end),[0]*6)
	for i in range(4):
		mp=movej(q_init,999999,10,(mp,))
		mp=movej(q_end,999999,10,(mp,))
	
	timestamp,curve_exe_js=execute((robot_client,mp))

	return get_acc(timestamp,curve_exe_js,q_d,joint)

def exec_motion_motoman(q_d,joint,displacement,MotionProgramFunc,robot,robot_client):
	###move joint at q_d configuration
	q_init=copy.deepcopy(q_d)
	q_end=copy.deepcopy(q_d)
	q_init[joint]+=displacement
	q_end[joint]-=displacement
	
	robot_client=movej(q_d,5,None,(robot_client,))

	j_init=jointtarget(np.degrees(q_init),[0]*6)
	j_end=jointtarget(np.degrees(q_end),[0]*6)
	for i in range(4):
		robot_client=movej(q_init,999999,None,(robot_client,))
		robot_client=movej(q_end,999999,None,(robot_client,))
	
	timestamp,curve_exe_js=execute((robot_client,))

	return get_acc(timestamp,curve_exe_js,q_d,joint)



def capture_acc(robot_name,robot,robot_client):
	MotionProgramFunc=MotionProgram

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
			qddot_max,_=exec(q_d,0,displacement,MotionProgramFunc,robot,robot_client)
			###update dict
			dict_table[(q2,q3)][0]=qddot_max
			dict_table[(q2,q3)][1]=qddot_max

			for joint in range(1,3):
				###move first q2 and q3
				qddot_max_p,qddot_max_n=exec_motion(q_d,joint,displacement,MotionProgramFunc,robot,robot_client)
				###update dict
				dict_table[(q2,q3)][2*joint]=qddot_max_p
				dict_table[(q2,q3)][2*joint+1]=qddot_max_n


	with open(r'test.txt','w+') as f:
		f.write(str(dict_table))
	pickle.dump(dict_table, open('test.pickle','wb'))

def capture_acc_collision(robot_name,robot,robot_client,tesseract_environment):
	
	MotionProgramFunc=MotionProgram

	resolution=0.3 ###rad
	displacement=0.02

	dict_table={}
	directions=[-1,1]

	#####################first & second joint acc both depends on second and third joint#####################################
	for q2 in np.arange(robot.lower_limit[1]+displacement+0.01,robot.upper_limit[1]-displacement-0.01,resolution):
		for q3 in np.arange(robot.lower_limit[2]+displacement+0.01,robot.upper_limit[2]-displacement-0.01,resolution):
			###check for collision
			if tesseract_environment.check_collision_single(robot_name,np.array([0,q2,q3,0,0,0])):
				continue
			###initialize keys, and desired pose
			dict_table[(q2,q3)]=[0]*6 		###[+j1,-j1,+j2,-j2,+j3,-j3]
			q_d=[0,q2,q3,0,0,0]

			#measure first joint first
			qddot_max,_=exec_motion(q_d,0,displacement,MotionProgramFunc,robot,robot_client)
			###update dict
			dict_table[(q2,q3)][0]=qddot_max
			dict_table[(q2,q3)][1]=qddot_max

			for joint in range(1,3):
				###move first q2 and q3
				qddot_max_p,qddot_max_n=exec_motion(q_d,joint,displacement,MotionProgramFunc,robot,robot_client)
				###update dict
				dict_table[(q2,q3)][2*joint]=qddot_max_p
				dict_table[(q2,q3)][2*joint+1]=qddot_max_n


	with open(r'test.txt','w+') as f:
		f.write(str(dict_table))
	pickle.dump(dict_table, open('test.pickle','wb'))


movej=movej_motoman
execute=execute_motoman
exec_motion=exec_motion_motoman

def main_abb():
	robot_name='ABB_6640_180_255'
	robot=robot_obj(robot_name,'config/abb_6640_180_255_robot_default_config.yml')
	robot_client=MotionProgramExecClient(base_url="http://192.168.55.1:80")
	ABB_6640_180_255_joint_names=["ABB_6640_180_255_joint_1","ABB_6640_180_255_joint_2","ABB_6640_180_255_joint_3","ABB_6640_180_255_joint_4","ABB_6640_180_255_joint_5","ABB_6640_180_255_joint_6"]
	ABB_6640_180_255_link_names=["ABB_6640_180_255_link_1","ABB_6640_180_255_link_2","ABB_6640_180_255_link_3","ABB_6640_180_255_link_4","ABB_6640_180_255_link_5","ABB_6640_180_255_link_6"]
	
	#Robot dictionaries, all reference by name
	robot_linkname={'ABB_6640_180_255':ABB_6640_180_255_link_names}
	robot_jointname={'ABB_6640_180_255':ABB_6640_180_255_joint_names}
	
	t=Tess_Env('config/urdf/abb_cell',robot_linkname,robot_jointname)

	capture_acc_collision(robot_name,robot,robot_client,t)

def main_motoman():
	robot_name='MA2010_A0'
	robot=robot_obj(robot_name,def_path='config/MA2010_A0_robot_default_config.yml',pulse2deg_file_path='config/MA2010_A0_pulse2deg.csv')
	robot_client=MotionProgramExecClient(ROBOT_CHOICE='RB1',pulse2deg=robot.pulse2deg)
	#link and joint names in urdf
	MA2010_link_names=["MA2010_base_link","MA2010_link_1_s","MA2010_link_2_l","MA2010_link_3_u","MA2010_link_4_r","MA2010_link_5_b","MA2010_link_6_t"]
	MA2010_joint_names=["MA2010_joint_1_s","MA2010_joint_2_l","MA2010_joint_3_u","MA2010_joint_4_r","MA2010_joint_5_b","MA2010_joint_6_t"]

	#Robot dictionaries, all reference by name
	robot_linkname={'MA2010_A0':MA2010_link_names}
	robot_jointname={'MA2010_A0':MA2010_joint_names}
	
	t=Tess_Env('config/urdf/motoman_cell',robot_linkname,robot_jointname)

	capture_acc_collision(robot_name,robot,robot_client,t)


if __name__ == '__main__':
	
	main_motoman()