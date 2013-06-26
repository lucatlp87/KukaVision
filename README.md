KukaVision
==========
************
INSTALLATION
All required libraries and thirdy-part libraries are listed in INSTALLATION_STEPS.txt.

****************
RUNNING THE CODE
In order to make the code running do the following steps
1. cd KukaVision
2. mkdir build && cd build
3. cmake ..
4. make -j3
5. mv ../scene.pcd .
6. ./KukaVision

***********
DESCRIPTION

The main goal of this project is to keep an acquired scene from Kinect as much close as possible to an assigned reference one. The algorithm is model based. Models are represented by point clouds acquired using the depth sensor insted of using CAD models. Models are extracted from the acquired scene using the Euclidean Clustering algorithm while they are represented by an OUR-CVFH signature. The signature is used to search the DB in order to get correspondences between clusters. 

The whole project is divided into three main steps.

- TRAINING STAGE

The training stage deals with the creation of the DB of objects models. 

All models are extracted from the acquired scene using the Euclidean Clustering algorithm. Before using the algorithm a pass-through filter is implemented in order to extract the dominant plane cluster (corresponding to the tabletop). In order to make the code running well set the acquisition camera position and the pass-through filter parameters so that that the acquired cloud as only one dominant plane cluster.
Once the cloud is acquired and clusterized, each extracted cluster is upsampled (using a MLS filter) and processed in order to obtain its OUR-CVFH signature. During these steps the user is required to choose if the considered cluster has to be saved or not (a visualizer allows a simple visual recognition of the cluster in the scene). If it the cluster has to be saved the user is asked to insert the name of the folder (in objects models DB) in which the model will be stored, together with the name of the file. For each model the user wants to represent, several acquisition have to be performed. Each acquisition will correspond to a different point of view of the object.
In each folder of the DB the follwoing items are stored:
		- a .pcd file corresponding to the cluster of each point of view (in <PointNormal> format)
		- a .pcd file corresponding to the OUR-CVFH signature of each cluster (in <VFHSignature308> format)
		- a .h5 file corresponding to a matrix (Flann format) containing OUR-CVFH signatures of all elements of the folder
		- a .index file corresponding to the Kd-Tree of the folder
		- a .list file containing paths to all .pcd files corresponding to OUR-CVFH signatures in the folder

- NEWSCENE STAGE

The second step deals with the definition and the storage of a reference scene. 

The first part of the procedure is the same of the previous step. The only difference is that, in the previouse case, the scene was made of only one object (the one of which the user was defining the model) while in this case the scene is composed by several objects.
Once all clusters are extracted from the acquired cloud, each of them is upsampled (using MLS again) and the corresponding OUR-CVFH signature is estimated. The signature is used to search the DB for a correspondence. If a correspondence is found the object become part of the scene. If not the user is asked if save the current cluster as a new model or ignore it. 
When all clusters are recognized (or ignored) the scene is saved in the DB dedicated to reference scenes (the user is asked to specify the name of the scene). The DB is composed of folders and each folder corresponds to a scene. In each folder the following items are stored:

	- a .pcd file corresponding to the cloud of the whole scene acquired from Kinect

	- a .txt file containing infos about objects part of the scene (in particular each object has an ID, a type and the 		  path to the corresponding folder in the objects models DB)

At last, a visualizer will open showing the whole scene and each recognized object with some information (type and reference pose)

- KUKAVISION STAGE

The last stage deals with a modified scene (that has to be modified in order to become as much close as possible to the reference one). The procedure is exactly the same of the previous step until the DB search section. In addition to this, the very first action allows the user to choose a reference scene from the DB.
Once all clusters are extracted and OUR-CVFH signatures are estimated a different DB search is implemented. In particular, for each cluster, the algorithm searches a correspondence only in the objects models subfolders saved in the .txt file of the chosen reference scene. If the the correspondence is found a transformation matrix between the current pose and the reference one is estimated. If not the object has to be thrown away, so a the transformation matrix is estimated in a different way. 
When all transformations are defined, the KUKA arm can operate grasping objects and moving them in a proper way.
