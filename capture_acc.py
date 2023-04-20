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
	mp.MoveAbsJ(abb.jointtarget(np.degrees(q),[0]*6),speeddata(v,9999999,9999999,999999),zonedata(False,z,1.5*z,1.5*z,0.15*z,1.5*z,0.15*z))
	return mp

def execute_abb(args):
	###execute function
	#args:(client,motionprogram)
	(client,mp)=args
	log_results = client.execute_motion_program(mp)

	return log_results.data[:,0],np.radians(log_results.data[:,2:8]), log_results.data[:,1]

def movej_motoman(q,v,z,args):
	###abb moveabsj function
	#q: joint angle in radians
	#v: tcp speed
	#z: blending zone
	#args: (client,)
	(client,)=args
	client.MoveJ(np.degrees(q),min(v,100),z)
	return client

name_map={'RB1':(0,6),'RB2':(6,12),'ST1':(12,14)}

def execute_motoman(args):
	###execute function
	#args:(client,)
	(client,)=args
	timestamp, curve_exe_js,job_line,_ = client.execute_motion_program()

	return timestamp-timestamp[0], curve_exe_js[:,name_map[client.ROBOT_CHOICE][0]:name_map[client.ROBOT_CHOICE][1]], job_line



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
	
	mp = MotionProgram()
	mp=movej_abb(q_d,200,0,(mp,))
	execute[robot.robot_name]((robot_client,mp))

	mp = MotionProgram()

	for i in range(4):
		mp=movej_abb(q_init,999999,zone,(mp,))
		mp=movej_abb(q_end,999999,zone,(mp,))
	
	timestamp,curve_exe_js,cmd_num=execute[robot.robot_name]((robot_client,mp))

	return get_acc(timestamp,curve_exe_js,joint)

def exec_motion_motoman(q_d,joint,displacement,robot,robot_client,zone=None):
	###move joint at q_d configuration
	q_init=copy.deepcopy(q_d)
	q_end=copy.deepcopy(q_d)
	q_init[joint]+=displacement
	q_end[joint]-=displacement
	
	robot_client=movej_motoman(q_d,5,None,(robot_client,))


	for i in range(4):
		robot_client=movej_motoman(q_init,999999,zone,(robot_client,))
		robot_client=movej_motoman(q_end,999999,zone,(robot_client,))
	
	timestamp,curve_exe_js,job_line=execute[robot.robot_name]((robot_client,))
	idx = np.absolute(job_line-2).argmin()
	start_idx=np.where(job_line==job_line[idx])[0][0]

	return get_acc(timestamp[start_idx:],curve_exe_js[start_idx:],joint)



def capture_acc(robot,robot_client,zone,displacement,resolution,q0_default=0):

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


	with open(r'test.txt','w+') as f:
		f.write(str(dict_table))
	pickle.dump(dict_table, open('test.pickle','wb'))

def capture_acc_456(robot,robot_client,zone,displacement,resolution,q_d=np.zeros(6)):
	acc=[]
	for q in range(3,6):
		qddot_max_p,qddot_max_n=exec_motion[robot.robot_name](q_d,q,displacement,robot,robot_client,zone)
		print(qddot_max_p,qddot_max_n)
		acc.append(qddot_max_p)
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


	acc_456=[]
	for q in range(3,6):
		qddot_max_p,qddot_max_n=exec_motion[robot.robot_name]([q0_default]+[0]*5,q,2*displacement,robot,robot_client,zone)
		acc_456.append(qddot_max_p)

	return dict_table, acc_456

#####ROBOT SPECIFIC MAPPING
execute={'MA2010_A0':execute_motoman,'MA1440_A0':execute_motoman,'ABB_6640_180_255':execute_abb,'ABB_1200_5_90':execute_abb}
exec_motion={'MA2010_A0':exec_motion_motoman,'MA1440_A0':exec_motion_motoman,'ABB_6640_180_255':exec_motion_abb,'ABB_1200_5_90':exec_motion_abb}
joint_names={'ABB_6640_180_255':["ABB_6640_180_255_joint_1","ABB_6640_180_255_joint_2","ABB_6640_180_255_joint_3","ABB_6640_180_255_joint_4","ABB_6640_180_255_joint_5","ABB_6640_180_255_joint_6"],\
			'ABB_1200_5_90':["ABB_1200_5_90_joint_1","ABB_1200_5_90_joint_2","ABB_1200_5_90_joint_3","ABB_1200_5_90_joint_4","ABB_1200_5_90_joint_5","ABB_1200_5_90_joint_6"],\
			'MA2010_A0':["MA2010_joint_1_s","MA2010_joint_2_l","MA2010_joint_3_u","MA2010_joint_4_r","MA2010_joint_5_b","MA2010_joint_6_t"],\
			'MA1440_A0':["MA1440_joint_1_s","MA1440_joint_2_l","MA1440_joint_3_u","MA1440_joint_4_r","MA1440_joint_5_b","MA1440_joint_6_t"]}
link_names={'ABB_6640_180_255':["ABB_6640_180_255_link_1","ABB_6640_180_255_link_2","ABB_6640_180_255_link_3","ABB_6640_180_255_link_4","ABB_6640_180_255_link_5","ABB_6640_180_255_link_6"],\
			'ABB_1200_5_90':["ABB_1200_5_90_link_1","ABB_1200_5_90_link_2","ABB_1200_5_90_link_3","ABB_1200_5_90_link_4","ABB_1200_5_90_link_5","ABB_1200_5_90_link_6"],\
			'MA2010_A0':["MA2010_base_link","MA2010_link_1_s","MA2010_link_2_l","MA2010_link_3_u","MA2010_link_4_r","MA2010_link_5_b","MA2010_link_6_t","weldgun","MA2010_filler"],\
			'MA1440_A0':["MA1440_base_link","MA1440_link_1_s","MA1440_link_2_l","MA1440_link_3_u","MA1440_link_4_r","MA1440_link_5_b","MA1440_link_6_t","scanner"]}

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

	capture_acc_collision(robot,robot_client,10,t)

def main_motoman():
	robot_name='MA2010_A0'
	robot=robot_obj(def_path='config/MA2010_A0_robot_default_config.yml',pulse2deg_file_path='config/MA2010_A0_pulse2deg.csv')
	robot_client=MotionProgramExecClient(ROBOT_CHOICE='RB1',pulse2deg=robot.pulse2deg)
	#link and joint names in urdf
	MA2010_link_names=["MA2010_base_link","MA2010_link_1_s","MA2010_link_2_l","MA2010_link_3_u","MA2010_link_4_r","MA2010_link_5_b","MA2010_link_6_t","weldgun","MA2010_filler"]
	MA2010_joint_names=["MA2010_joint_1_s","MA2010_joint_2_l","MA2010_joint_3_u","MA2010_joint_4_r","MA2010_joint_5_b","MA2010_joint_6_t"]

	#Robot dictionaries, all reference by name
	robot_linkname={'MA2010_A0':MA2010_link_names}
	robot_jointname={'MA2010_A0':MA2010_joint_names}
	
	t=Tess_Env('config/urdf/motoman_cell',robot_linkname,robot_jointname)

	displacement=0.03
	resolution=0.3
	zone=None
	q0_default=0.17
	# capture_acc_collision(robot,robot_client,zone,displacement,resolution,t,q0_default=q0_default)
	print(capture_acc_456(robot,robot_client,zone,displacement,resolution))
	

def osc_test():
	robot_name='MA2010_A0'
	robot=robot_obj(robot_name,def_path='config/MA2010_A0_robot_default_config.yml',pulse2deg_file_path='config/MA2010_A0_pulse2deg.csv')
	robot_client=MotionProgramExecClient(ROBOT_CHOICE='RB1',pulse2deg=robot.pulse2deg)

	q_d=np.zeros(6)
	displacement=0.03

	zone=None
	joint=2
	###move joint at q_d configuration
	q_init=copy.deepcopy(q_d)
	q_end=copy.deepcopy(q_d)
	q_init[joint]+=displacement
	q_end[joint]-=displacement
	
	robot_client=movej_motoman(q_d,5,None,(robot_client,))

	for i in range(4):
		robot_client=movej_motoman(q_init,999999,zone,(robot_client,))
		robot_client=movej_motoman(q_end,999999,zone,(robot_client,))
	
	timestamp,curve_exe_js, job_line=execute((robot_client,))
	idx = np.absolute(job_line-2).argmin()
	start_idx=np.where(job_line==job_line[idx])[0][0]

	print(get_acc(timestamp[start_idx:],curve_exe_js[start_idx:],joint))
	plt.title('Joint %i Oscillation'%(joint+1))
	plt.xlabel('Time (s)')
	plt.ylabel('Joint Angle (rad)')
	plt.plot(timestamp[start_idx:],curve_exe_js[start_idx:,joint])
	plt.show()


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

	robot_client_map={'MA2010_A0':motoman.MotionProgramExecClient(ROBOT_CHOICE='RB1',pulse2deg=robot.pulse2deg,IP=args.robot_ip),\
						'MA1440_A0':motoman.MotionProgramExecClient(ROBOT_CHOICE='RB2',pulse2deg=robot.pulse2deg,IP=args.robot_ip),\
						'ABB_6640_180_255':abb.MotionProgramExecClient(base_url="http://"+args.robot_ip+":80"),\
						'ABB_1200_5_90':abb.MotionProgramExecClient(base_url="http://"+args.robot_ip+":80")}

	if type(args.urdf_path) is not None:
		t=Tess_Env(args.urdf_path,{args.robot_name:link_names[args.robot_name]},{args.robot_name:joint_names[args.robot_name]})
		t.t_env.setState(joint_names['MA2010_A0'], np.array([0,0,1.57,0,0,0]))
		dict_table,acc456=capture_acc_collision(robot,robot_client_map[args.robot_name],\
			args.zone,args.displacement,args.resolution,t,q0_default=args.q0_default)
	else:
		dict_table,acc456=capture_acc(robot,robot_client_map[args.robot_name],\
			args.zone,args.displacement,args.resolution,q0_default=args.q0_default)

	with open(r'test.txt','w+') as f:
		f.write(str(dict_table))
	pickle.dump(dict_table, open(args.output_dir+'test.pickle','wb'))

if __name__ == '__main__':
	# osc_test()
	# main_motoman()
	main()