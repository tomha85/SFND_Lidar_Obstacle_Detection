/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree

struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)	 
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void insertHelper(Node** node,uint depth, pcl::PointXYZI point,int id)
	{
		if(*node==NULL)
		{
			*node= new Node(point,id);
		}
		else
		{
			int cd = depth % 2 ;
			if (cd == 0)
			{
				if(point.x<((*node)->point.x))
					insertHelper(&((*node)->left),depth+1,point,id);
				else
					insertHelper(&((*node)->right),depth+1,point,id);
			else
			{
				if(point.y<(*node)->point.y)
					insertHelper(&((*node)->left),depth+1,point,id);
				else
					insertHelper(&((*node)->right),depth+1,point,id);
			}
			
			
		}
			
	}
	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id);

	}
	void searchHelper(pcl::PointXYZI target, Node* node,int depth,float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{

			if( (node->point.x>=(target.x-distanceTol)&&node->point.x<=(target.x+distanceTol)) && (node->point.y>=(target.y-distanceTol)&&node->point.y<=(target.y+distanceTol)))
			{
				float distance=sqrt((node->point.x-target.x)*(node->point.x-target.x)+(node->point.y-target.y)*(node->point.y-target.y));
				if(distance <= distanceTol)
					ids.push_back(node->id);
			}
			if(depth % 2==0)
			{
					if((target.x-distanceTol)<node->point.x)
						searchHelper(target,node->left,depth+1,distanceTol,ids);
					if((target.x+distanceTol)>node->point.x)
						searchHelper(target,node->right,depth+1,distanceTol,ids);
			}
			else
			{	
					if((target.y-distanceTol)<node->point.y)
						searchHelper(target,node->left,depth+1,distanceTol,ids);
					if((target.y+distanceTol)>node->point.y)
						searchHelper(target,node->right,depth+1,distanceTol,ids);
			}
						
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}
	

};



