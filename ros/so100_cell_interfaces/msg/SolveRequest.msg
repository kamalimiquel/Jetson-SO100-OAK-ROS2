# One correlated IK request. The id is echoed in SolveResult, so a consumer knows which target a solution
# answers instead of hoping the last message on a topic was its own.
string request_id
geometry_msgs/PoseStamped target
float64 max_pos_err_mm 0.5
float64 max_ori_err_deg 2.0
