import abb_motion_program_exec as abb
import dx200_motion_program_exec_client as motoman

from pandas import *
import json, pickle, copy, argparse
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
	mp.MoveAbsJ(abb.jointtarget(np.degrees(q),[0]*6),abb.speeddata(v,9999999,9999999,999999),abb.zonedata(False,z,1.5*z,1.5*z,0.15*z,1.5*z,0.15*z))
	return mp

def movej_motoman(q,v,z,args):
	###abb moveabsj function
	#q: joint angle in radians
	#v: tcp speed
	#z: blending zone
	#args: (client,)
	(mp,)=args
	mp.MoveJ(np.degrees(q),min(v,100),z)
	return mp


def execute_abb(args):
	###execute function
	#args:(client,motionprogram)
	(client,mp)=args
	log_results = client.execute_motion_program(mp)

	return log_results.data[:,0],np.radians(log_results.data[:,2:8]), log_results.data[:,1]


def execute_motoman(args):
	###execute function
	#args:(client,motionprogram)
	(client,mp)=args
	timestamp, curve_exe_js,job_line,_ = client.execute_motion_program(mp)

	return timestamp-timestamp[0], curve_exe_js[:,name_map[mp.ROBOT_CHOICE][0]:name_map[mp.ROBOT_CHOICE][1]], job_line



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


def get_acc(timestamp,curve_exe_js,joint):
	###filter
	timestamp, curve_exe_js=lfilter(timestamp, curve_exe_js)
	###get qdot, qddot
	qdot_all=np.gradient(curve_exe_js,axis=0)/np.tile([np.gradient(timestamp)],(6,1)).T
	qddot_all=np.gradient(qdot_all,axis=0)/np.tile([np.gradient(timestamp)],(6,1)).T

	qddot_sorted=np.sort(qddot_all[:,joint])

	qddot_max_p=np.average(qddot_sorted[-5:])
	qddot_max_n=-np.average(qddot_sorted[:5])

	return qddot_max_p, qddot_max_n

def exec_motion_abb(q_d,joint,displacement,robot,robot_client,zone=10):

	###move joint at q_d configuration
	q_init=copy.deepcopy(q_d)
	q_end=copy.deepcopy(q_d)
	q_init[joint]+=displacement
	q_end[joint]-=displacement
	
	mp = abb.MotionProgram()
	mp=movej[robot.robot_name](q_d,200,0,(mp,))
	execute[robot.robot_name]((robot_client,mp))

	mp = abb.MotionProgram()

	for i in range(4):
		mp=movej[robot.robot_name](q_init,999999,zone,(mp,))
		mp=movej[robot.robot_name](q_end,999999,zone,(mp,))
	
	timestamp,curve_exe_js,cmd_num=execute[robot.robot_name]((robot_client,mp))

	return get_acc(timestamp,curve_exe_js,joint)

def exec_motion_motoman(q_d,joint,displacement,robot,robot_client,zone=None):
	###move joint at q_d configuration
	q_init=copy.deepcopy(q_d)
	q_end=copy.deepcopy(q_d)
	q_init[joint]+=displacement
	q_end[joint]-=displacement
	

	mp = motoman.MotionProgram(pulse2deg=robot.pulse2deg,ROBOT_CHOICE=robot_choice[robot.robot_name])
	mp=movej[robot.robot_name](q_d,5,None,(mp,))
	execute[robot.robot_name]((robot_client,mp))

	mp = motoman.MotionProgram(pulse2deg=robot.pulse2deg,ROBOT_CHOICE=robot_choice[robot.robot_name])
	for i in range(4):
		mp=movej[robot.robot_name](q_init,999999,zone,(mp,))
		mp=movej[robot.robot_name](q_end,999999,zone,(mp,))
	
	timestamp,curve_exe_js,job_line=execute[robot.robot_name]((robot_client,mp))
	
	return get_acc(timestamp,curve_exe_js,joint)

def robot_client_map(robot,robot_ip):
	if 'MA' in robot.robot_name:
		return motoman.MotionProgramExecClient(IP=robot_ip)

	if 'ABB' in robot.robot_name:
		return abb.MotionProgramExecClient(base_url="http://"+robot_ip+":80")



def capture_acc(robot,robot_client,zone,displacement,resolution,q0_default=0):
	###JOINT 456 FIRST
	acc_456=[]
	for q in range(3,6):
		qddot_max_p,qddot_max_n=exec_motion[robot.robot_name]([q0_default]+[0]*5,q,2*displacement,robot,robot_client,zone)
		acc_456.append(qddot_max_p)
		
	dict_table={}
	directions=[-1,1]

	#####################first & second joint acc both depends on second and third joint#####################################
	for q2 in np.arange(robot.lower_limit[1]+displacement+0.01,robot.upper_limit[1]-displacement-0.01,resolution):
		for q3 in np.arange(robot.lower_limit[2]+displacement+0.01,robot.upper_limit[2]-displacement-0.01,resolution):
			###initialize keys, and desired pose

			dict_table[(q2,q3)]=[0]*6 		###[+j1,-j1,+j2,-j2,+j3,-j3]
			q_d=[q0_default,q2,q3,0,0,0]

			#measure first joint first
			qddot_max,_=exec_motion[robot.robot_name](q_d,0,displacement,robot,robot_client,zone)
			###update dict
			dict_table[(q2,q3)][0]=qddot_max
			dict_table[(q2,q3)][1]=qddot_max

			for joint in range(1,3):
				###move first q2 and q3
				qddot_max_p,qddot_max_n=exec_motion[robot.robot_name](q_d,joint,displacement,robot,robot_client,zone)
				###update dict
				dict_table[(q2,q3)][2*joint]=qddot_max_p
				dict_table[(q2,q3)][2*joint+1]=qddot_max_n

	return dict_table

	

def capture_acc_456(robot,robot_client,zone,displacement,resolution,q_d=np.zeros(6)):
	acc=[]
	for q in range(3,6):
		qddot_max_p,qddot_max_n=exec_motion[robot.robot_name](q_d,q,displacement,robot,robot_client,zone)
		print(qddot_max_p,qddot_max_n)
		acc.append((qddot_max_p+qddot_max_n)/2)

	np.savetxt('test456.txt',acc,delimiter=',')

	return acc

def capture_acc_collision(robot,robot_client,zone,displacement,resolution,tesseract_environment,q0_default=0):

	dict_table={}
	directions=[-1,1]

	#####################first & second joint acc both depends on second and third joint#####################################
	for q2 in np.arange(robot.lower_limit[1]+displacement+0.01,robot.upper_limit[1]-displacement-0.01,resolution):
		for q3 in np.arange(robot.lower_limit[2]+displacement+0.01,robot.upper_limit[2]-displacement-0.01,resolution):
			###check for collision
			if tesseract_environment.check_collision_single(robot.robot_name,np.array([0,q2,q3,0,0,0])):
				continue
			###initialize keys, and desired pose
			dict_table[(q2,q3)]=[0]*6 		###[+j1,-j1,+j2,-j2,+j3,-j3]
			q_d=[q0_default,q2,q3,0,0,0]

			#measure first joint first
			qddot_max,_=exec_motion[robot.robot_name](q_d,0,displacement,robot,robot_client,zone)
			###update dict
			dict_table[(q2,q3)][0]=qddot_max
			dict_table[(q2,q3)][1]=qddot_max

			for joint in range(1,3):
				###move first q2 and q3
				qddot_max_p,qddot_max_n=exec_motion[robot.robot_name](q_d,joint,displacement,robot,robot_client,zone)
				###update dict
				dict_table[(q2,q3)][2*joint]=qddot_max_p
				dict_table[(q2,q3)][2*joint+1]=qddot_max_n


	return dict_table

#####ROBOT SPECIFIC MAPPING

name_map={'RB1':(0,6),'RB2':(6,12)}
robot_choice={'MA2010_A0':'RB1','MA1440_A0':'RB2'}

execute={'MA2010_A0':execute_motoman,'MA1440_A0':execute_motoman,'ABB_6640_180_255':execute_abb,'ABB_1200_5_90':execute_abb}
movej={'MA2010_A0':movej_motoman,'MA1440_A0':movej_motoman,'ABB_6640_180_255':movej_abb,'ABB_1200_5_90':movej_abb}
exec_motion={'MA2010_A0':exec_motion_motoman,'MA1440_A0':exec_motion_motoman,'ABB_6640_180_255':exec_motion_abb,'ABB_1200_5_90':exec_motion_abb}
joint_names={'ABB_6640_180_255':["ABB_6640_180_255_joint_1","ABB_6640_180_255_joint_2","ABB_6640_180_255_joint_3","ABB_6640_180_255_joint_4","ABB_6640_180_255_joint_5","ABB_6640_180_255_joint_6"],\
			'ABB_1200_5_90':["ABB_1200_5_90_joint_1","ABB_1200_5_90_joint_2","ABB_1200_5_90_joint_3","ABB_1200_5_90_joint_4","ABB_1200_5_90_joint_5","ABB_1200_5_90_joint_6"],\
			'MA2010_A0':["MA2010_joint_1_s","MA2010_joint_2_l","MA2010_joint_3_u","MA2010_joint_4_r","MA2010_joint_5_b","MA2010_joint_6_t"],\
			'MA1440_A0':["MA1440_joint_1_s","MA1440_joint_2_l","MA1440_joint_3_u","MA1440_joint_4_r","MA1440_joint_5_b","MA1440_joint_6_t"]}
link_names={'ABB_6640_180_255':["ABB_6640_180_255_link_1","ABB_6640_180_255_link_2","ABB_6640_180_255_link_3","ABB_6640_180_255_link_4","ABB_6640_180_255_link_5","ABB_6640_180_255_link_6"],\
			'ABB_1200_5_90':["ABB_1200_5_90_link_1","ABB_1200_5_90_link_2","ABB_1200_5_90_link_3","ABB_1200_5_90_link_4","ABB_1200_5_90_link_5","ABB_1200_5_90_link_6"],\
			'MA2010_A0':["MA2010_base_link","MA2010_link_1_s","MA2010_link_2_l","MA2010_link_3_u","MA2010_link_4_r","MA2010_link_5_b","MA2010_link_6_t","weldgun","MA2010_filler"],\
			'MA1440_A0':["MA1440_base_link","MA1440_link_1_s","MA1440_link_2_l","MA1440_link_3_u","MA1440_link_4_r","MA1440_link_5_b","MA1440_link_6_t","scanner"]}


def main():
	#Accept the names of the webcams and the nodename from command line
	parser = argparse.ArgumentParser(description="Robot Acceleration Capture")
	parser.add_argument("--robot-name",type=str,required=True)
	parser.add_argument("--robot-info-file",type=str,required=True)
	parser.add_argument("--pulse2deg-file",type=str,default='config/MA2010_A0_pulse2deg.csv')
	parser.add_argument("--displacement", type=float, default=0.03, help="oscillation amplitude (rad)")
	parser.add_argument("--resolution", type=float, default=0.3, help="Sampling Joint Resolution (rad)")
	parser.add_argument("--zone", type=float, default=None, help="blending zone")
	parser.add_argument("--q0-default", type=float, default=0., help="default joint 1 position")
	parser.add_argument("--robot-ip",type=str,required=True)
	parser.add_argument("--urdf-path",type=str,default=None)
	parser.add_argument("--output-dir",type=str,default='results/')
	args, _ = parser.parse_known_args()

	robot=robot_obj(args.robot_name,def_path=args.robot_info_file,pulse2deg_file_path=args.pulse2deg_file)

	
	if args.urdf_path is not None:
		t=Tess_Env(args.urdf_path,{args.robot_name:link_names[args.robot_name]},{args.robot_name:joint_names[args.robot_name]})
		###hard coded robot first joint position to avoid collision
		t.t_env.setState(joint_names['MA2010_A0'], np.array([0,0,1.57,0,0,0]))

		capture_acc_456(robot,robot_client_map(robot,args.robot_ip),args.zone,args.displacement,args.resolution)

		dict_table=capture_acc_collision(robot,robot_client_map(robot,args.robot_ip),\
			args.zone,args.displacement,args.resolution,t,q0_default=args.q0_default)
	else:
		capture_acc_456(robot,robot_client_map(robot,args.robot_ip),args.zone,args.displacement,args.resolution)

		dict_table=capture_acc(robot,robot_client_map(robot,args.robot_ip),\
			args.zone,args.displacement,args.resolution,q0_default=args.q0_default)

	with open(r'test.txt','w+') as f:
		f.write(str(dict_table))
	pickle.dump(dict_table, open(args.output_dir+'test.pickle','wb'))

if __name__ == '__main__':
	# osc_test()
	# main_motoman()
	main()