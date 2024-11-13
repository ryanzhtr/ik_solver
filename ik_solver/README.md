ik_solver调用了TRAC IK作为机械臂逆运动学的计算器，适用于6-DOF和7-DOF的机械臂

ik_solver功能包依赖于NLopt Library，安装过程参考 “https://github.com/stevengj/nlopt“

对于不同的机械臂需要更换../src/ik_solver/launch下的urdf文件，并根据urdf文件修改iksolve.launch.py中的

`urdf_file = os.path.join(pkg_share, 'launch', 'xarm7.urdf')`

`DeclareLaunchArgument('chain_start', default_value='link_base')`

`DeclareLaunchArgument('chain_end', default_value='link7')`

其中chain_start和chain_end构成完整的KDL::Chain
