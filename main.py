from bvh_skeleton import smartbody_skeleton_customer
import numpy as np
import os
import pdb
import json
'''
把json格式的文件转为bvh格式的文件
'''
def mkdir_if_not_exists(path):
	if not os.path.exists(path):
		os.makedirs(path)

fdr_path = r'./json/15nodes'
save_path = fdr_path.replace('json','bvh') 
mkdir_if_not_exists(save_path)
files = sorted(next(os.walk(fdr_path))[2]) 
for file in files:
	file_path = os.path.join(fdr_path,file)
	with open(file_path,'r') as f:
		data = json.load(f)
	data = np.array(data['data']) 
	data = data.reshape(data.shape[0],-1,3) 
	new_points = np.zeros((data.shape[0],data.shape[1],data.shape[2]),dtype = np.float32) 
	new_points[:,:,0], new_points[:,:,1], new_points[:,:,2] = data[:,:,1],data[:,:,2],data[:,:,0]
	if '20nodes' in fdr_path:
		SmartBody_skeleton = smartbody_skeleton_customer.SmartBodySkeleton20()
	elif '15nodes' in fdr_path:
		SmartBody_skeleton = smartbody_skeleton_customer.SmartBodySkeleton15()
	else:
		print('select wrong, manully set')
	save_file_path = os.path.join(save_path,file.replace('json','bvh'))  
	SmartBody_skeleton.poses2bvh(new_points,output_file=save_file_path)  
	print(file)
