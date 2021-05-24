from bvh_skeleton import smartbody_skeleton_customer
import numpy as np
import os
import pdb
import json
#把json文件转为bvh，
#只需要修改lines14,路径中的15nodes和20nodes，以及arm curl文件夹


def mkdir_if_not_exists(path):
	if not os.path.exists(path):
		os.makedirs(path)

# fdr_path = r'../dataset/20nodes/arm curl'
fdr_path = r'../dataset/20nodes/backflip'
save_path = fdr_path.replace('dataset','bvh')  #fdr_path是一个字符串，replace是替换改字符串中个部分
mkdir_if_not_exists(save_path)
files = sorted(next(os.walk(fdr_path))[2]) 
for file in files:
	file_path = os.path.join(fdr_path,file)
	with open(file_path,'r') as f:
		data = json.load(f)
	# pdb.set_trace()
	data = np.array(data['data']) #嵌套列表转为2维arr,（259，60）
	data = data.reshape(data.shape[0],-1,3) #改变列表的形状（259，20，3）
	# pdb.set_trace()
	# index = data[:,0]
	# videodata = data[:,1:]
	# videodata = videodata.reshape(videodata.shape[0],-1,3)
	# points = np.array(videodata,dtype=np.float32)
	# pdb.set_trace()
	# convert axis
	new_points = np.zeros((data.shape[0],data.shape[1],data.shape[2]),dtype = np.float32) #建立一个形状相同全为0的数组
	new_points[:,:,0], new_points[:,:,1], new_points[:,:,2] = data[:,:,1],data[:,:,2],data[:,:,0]

	if '20nodes' in fdr_path:
		SmartBody_skeleton = smartbody_skeleton_customer.SmartBodySkeleton20()
	elif '15nodes' in fdr_path:
		SmartBody_skeleton = smartbody_skeleton_customer.SmartBodySkeleton15()
	else:
		print('select wrong, manully set')
	# SmartBody_skeleton = smartbody_skeleton_customer.SmartBodySkeleton20()  #实例化类
	save_file_path = os.path.join(save_path,file.replace('json','bvh'))  
	SmartBody_skeleton.poses2bvh(new_points,output_file=save_file_path)  #调用类中的函数
	# SmartBody_skeleton.poses2bvh(位置参数,关键字参数)  

	print(file)
