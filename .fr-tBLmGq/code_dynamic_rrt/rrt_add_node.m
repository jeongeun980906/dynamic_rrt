function rrt_struct = rrt_add_node(rrt_struct, pos, u, parent, status, time)

rrt_struct.nr_node = rrt_struct.nr_node + 1;
idx = rrt_struct.nr_node;

rrt_struct.tree.pos{idx}    = pos;
rrt_struct.tree.u{idx}      = u;
rrt_struct.tree.parent{idx} = parent;
rrt_struct.tree.status{idx} = status;
rrt_struct.tree.color{idx}  = rand(1, 3);
rrt_struct.tree.time{idx}   = time;

