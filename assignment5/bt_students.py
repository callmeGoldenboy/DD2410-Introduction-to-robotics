#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

"""class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# go to door until at door
		b0 = pt.composites.Selector(
			name="Go to door fallback",
			children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
		)

		# tuck the arm
		b1 = tuckarm()

		# go to table
		b2 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		)

		# move to chair
		b3 = pt.composites.Selector(
			name="Go to chair fallback",
			children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
		)

		# lower head
		b4 = movehead("down")

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)"""

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# tuck the arm
		"""b1 = tuckarm()
		# lower head
		b2 = movehead("down")

		b3 = pick_cube()
		b4 = move_towards_table()
		b5 = place_cube()"""

		picking_seq = RSequence(name="Pick sequence", children=[tuckarm(),movehead("down"),pick_cube()])
		rotation = pt.composites.Selector("Rotate", children = [counter(28,"Rotational counter"),go("Rotation",0,-1)])
		translation = pt.composites.Selector("Translate",children=[counter(11,"Translational counter"),go("Translation",0.8,0)])
		move_to_table2 = RSequence(name="Move to second table",children=[rotation,translation])
		place_behaviour = place_cube()
		check_if_placed = is_placed()
		rotation_back = pt.composites.Selector("Rotate back", children = [counter(28," Back rotation counter"),go("Rotation_back",0,-1)])
		translation_back = pt.composites.Selector("Translate back",children=[counter(11,"Back translation counter"),go("Translation_back",0.8,0)])
		move_back_to_table1 = RSequence(name="Move back to first table", children=[rotation_back,translation_back])
		check_and_move_back = pt.composites.Selector("Check if cube is placed on table2, return back if not placed", children=[check_if_placed,move_back_to_table1])

		# become the tree
		main_sequence = RSequence(name="Main sequence", children=[picking_seq,move_to_table2,place_behaviour,check_and_move_back])
		#tree = pt.composites.Selector("Main selector", children=[main_sequence,move_back_to_table1])
		super(BehaviourTree, self).__init__(main_sequence)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)



if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
