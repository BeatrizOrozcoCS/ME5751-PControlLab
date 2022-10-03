# Binary search tree class and bsTree_node class
# @chang@cpp.edu

#class bsTree_node, wrapping for any class object
class bsTree_node:
	def __init__(self, obj=None, value=-1.0):
		self.obj = obj
		self.val = value
		self.left = None
		self.right = None
		self.parent = None

	def __del__(self):
		pass

# binary search tree class
class bsTree:
	def __init__(self):
		self.dummyRoot = bsTree_node(obj = None, value = 10000000) #dummy root that will never be removed
		self.nodeCount = 0

	#insert a bsTreeNode to binary search tree
	def insert(self,treeNode):
		self._insert(self.dummyRoot, treeNode)
		self.nodeCount += 1

	#find the node with minimum value, (most left child)
	def findMinNode(self):
		return self._findMinNode(self.dummyRoot)

	#remove a bsTreeNode
	def remove(self, treeNode):
		assert (treeNode.parent != None),"You cannot remove the dummy node, Check your program!"

		if(treeNode.left == None or treeNode.right==None):
			childNode = treeNode.left if (treeNode.left is not None) else treeNode.right
			if(treeNode == treeNode.parent.right):
				treeNode.parent.right = childNode
			else:
				treeNode.parent.left = childNode

			childNode.parent = treeNode.parent

		else:
			swapNode = self._findMinNode(node=treeNode.right)
			
			treeNode.right.parent = swapNode
			treeNode.left.parent = swapNode

			swapNode.parent.left = None
			swapNode.parent = treeNode.parent

			if(treeNode.val < treeNode.parent.val):
				treeNode.parent.left = swapNode
			else:
				treeNode.parent.right = swapNode

		self.nodeCount -= 1
		del treeNode

	#non-recursive insert function
	def _insert(self, root, treeNode):
		x = root
		p = root
		while x is not None:
			p = x
			if(treeNode.val < x.val):
				x=x.left
			else:
				x=x.right

		#insertion should happen at left side of p
		if(treeNode.val<p.val):
			p.left = treeNode
		else:
			p.right = treeNode

		treeNode.parent = p

	def _findMinNode(self,node):
		root = node
		while (root.left!=None):
			root = root.left
		return root

