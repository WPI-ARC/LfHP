Learning Trajectory Cost Functions From Human Preferences
============

The code is designed to support the research in LfHP. To run the code use pr2_hereshow_pick_traj.launch file.

The code implements the GUI to capture the human preferences by displaying precomputed trajectories for a particular environment in RViz and allowing a user to choose between the two, which one is better. The ranking of trajectories is computed based on the pairwise human decisions. The decision trees are used on a set of features to compute the cost function.

