/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id; // 0 -> 10
	Node* left; // a gauche du noeud
	Node* right; // a droite du noeud

	// function for creating a brand new node, when we can give in this brand new point in a band new Id
	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	// take a brand new point
	// point representation : vector of floats : the first element of th vetor is going to be your x component
	// and the second element going to be the y component.
	// Id = unique identifier for the point : just a andex that it references in the point cloud : 11 points / 2D data : 0 ->10

	// insert fonction : going to be recursive : le but est de parcourir l'arbre jusqu'a atteindre noeud NULL, une fois atteind le noeud prend un Id et le NULL node est tranféré en profondeur
	
	void insertHelper (Node** node, uint depth, std::vector<float> point, int id) // Node** double pointer is just a memory adress : 
	// memory adress of the pointer node that we are currently in the tree
	{
		//Tree is empty : verification si le node est NULL, *node est ce que la racine doit commencer
		if (*node==NULL) // we deference it to actually see what it's value is, in this case : Root pointer : new data that you should be poiting at now
			*node = new Node (point,id); // reasign this node in the tree  + assign this brand new node
		else
		{	// traversing th tree
			//calculate current dim
			uint cd =depth % 2; //(even or odd)
			// if even -> X-value comparaison

			// cd = 0 ou 1, if cd=0 : look at the x-values, if cd=1 : look at the y-values
			if (point[cd] < ((*node)-> point[cd]))
				insertHelper (&((*node)->left), depth+1, point, id);
			else 
				insertHelper(&((*node)-> right), depth+1,point,id);
		}
		// insertHelper is terminate when its hits a null node
	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		// call insertHelper : recursive fonction : able to reassign thi node in the tree.
		//&root : passing the memory adresse for root, je commence a root 
		//depth start à 0,
		insertHelper (&root,0,point,id);

	}


	void searchHelper (std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		
		if (node !=NULL)
		{
			if ((node ->point[0] >= target[0] - distanceTol && // check for x-axis
				node->point[0] <= target[0] + distanceTol) &&
				(node -> point[1] >= target[1] - distanceTol && // check for y-axis
				node -> point[1] <= target[1] + distanceTol)) 
					{
						float distance = sqrt((node->point[0] - target[0]) * (node->point[0]- target[0]) +
										(node ->point[1]- target[1]) * (node->point[1] - target[1]));
						if (distance < distanceTol) 
							ids.push_back(node->id);
					}
					

			//check accross boundary
			if((target[depth%2]-distanceTol)<node -> point[depth%2])
				searchHelper(target, node -> left, depth+1,distanceTol,ids);
			if ((target[depth%2] +distanceTol)>node-> point[depth%2])
				searchHelper (target, node ->right, depth+1, distanceTol,ids);
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper (target,root,0, distanceTol, ids);
		return ids;
	}
	

};




